"""Pluggable image-frame to H264/RTP subprocess encoders.

The ROS-facing package is intentionally hardware-neutral.  FFmpeg is the
portable software default; GStreamer element factories and properties are
configuration so deployments can select a vendor hardware pipeline without
putting device detection or topic names in this module.
"""

from __future__ import annotations

from collections import deque
from fractions import Fraction
import json
import re
import signal
import subprocess
import threading
from typing import Callable, Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

from ros_image_rtp_adapter.h264 import AnnexBAccessUnits

PropertyValue = Union[str, int, float, bool]
PropertyInput = Union[str, Mapping[str, PropertyValue]]
ArgumentInput = Union[str, Sequence[str]]

_ELEMENT_NAME = re.compile(r"^[A-Za-z0-9_.+-]+$")
_PROPERTY_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_-]*$")
_RAW_INPUTS: Mapping[str, Tuple[str, str, int]] = {
    "rgb8": ("rgb24", "rgb", 3),
    "bgr8": ("bgr24", "bgr", 3),
    "rgba8": ("rgba", "rgba", 4),
    "bgra8": ("bgra", "bgra", 4),
    "mono8": ("gray", "gray8", 1),
}


def _parse_properties(value: PropertyInput, parameter_name: str) -> Dict[str, PropertyValue]:
    if isinstance(value, str):
        try:
            parsed = json.loads(value or "{}")
        except json.JSONDecodeError as exc:
            raise ValueError(f"{parameter_name} must contain a JSON object: {exc}") from exc
    else:
        parsed = dict(value)
    if not isinstance(parsed, dict):
        raise ValueError(f"{parameter_name} must contain a JSON object")

    properties: Dict[str, PropertyValue] = {}
    for key, property_value in parsed.items():
        if not isinstance(key, str) or not _PROPERTY_NAME.fullmatch(key):
            raise ValueError(f"{parameter_name} contains an invalid property name: {key!r}")
        if not isinstance(property_value, (str, int, float, bool)):
            raise ValueError(
                f"{parameter_name}.{key} must be a string, number, or boolean"
            )
        if isinstance(property_value, str) and "\x00" in property_value:
            raise ValueError(f"{parameter_name}.{key} must not contain NUL")
        properties[key] = property_value
    return properties


def _parse_arguments(value: ArgumentInput, parameter_name: str) -> List[str]:
    if isinstance(value, str):
        try:
            parsed = json.loads(value or "[]")
        except json.JSONDecodeError as exc:
            raise ValueError(f"{parameter_name} must contain a JSON array: {exc}") from exc
    else:
        parsed = list(value)
    if not isinstance(parsed, list) or not all(isinstance(item, str) for item in parsed):
        raise ValueError(f"{parameter_name} must contain a JSON array of strings")
    if any("\x00" in item for item in parsed):
        raise ValueError(f"{parameter_name} must not contain NUL")
    return parsed


def _validate_element_name(value: str, parameter_name: str) -> str:
    if not _ELEMENT_NAME.fullmatch(value):
        raise ValueError(
            f"{parameter_name} must be a GStreamer element factory name, got {value!r}"
        )
    return value


def _format_number(value: float) -> str:
    return str(int(value)) if float(value).is_integer() else str(value)


def normalize_input_format(value: str) -> str:
    normalized = value.strip().lower()
    if normalized == "jpeg":
        return normalized
    if normalized not in _RAW_INPUTS:
        raise ValueError(
            "input_format must be one of: jpeg, " + ", ".join(_RAW_INPUTS)
        )
    return normalized


def packed_frame_bytes(input_format: str, width: int, height: int) -> int:
    normalized = normalize_input_format(input_format)
    if normalized == "jpeg":
        raise ValueError("JPEG frames are variable length")
    return int(width) * int(height) * _RAW_INPUTS[normalized][2]


class SubprocessRtpEncoder:
    """Common supervised stdin/subprocess lifecycle for encoder backends."""

    def __init__(self) -> None:
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._runtime_validated = False
        self._stderr_tail = deque(maxlen=20)
        self._access_unit_callback = None
        self._source_stamps = deque()
        self._metadata_lock = threading.Lock()

    @property
    def running(self) -> bool:
        proc = self._proc
        return proc is not None and proc.poll() is None

    @property
    def diagnostic(self) -> str:
        return "\n".join(self._stderr_tail)

    def start(self) -> None:
        with self._lock:
            if self._proc is not None:
                return
            self._launch_locked()

    def preflight(self) -> None:
        """Validate the configured backend without allocating an encoder."""

        with self._lock:
            if self._runtime_validated:
                return
            self.validate_runtime()
            self._runtime_validated = True

    def stop(self) -> None:
        with self._lock:
            proc = self._proc
            self._proc = None
        self._stop_process(proc)

    def set_access_unit_callback(self, callback: Callable[[bytes, int], None]) -> None:
        if not isinstance(self, FFmpegRtpEncoder):
            raise ValueError("ROS H264 preview requires the FFmpeg encoder backend")
        if self._proc is not None:
            raise RuntimeError("configure H264 preview before starting the encoder")
        self._access_unit_callback = callback

    def write_frame(self, frame: bytes, source_stamp_ns: Optional[int] = None) -> None:
        with self._lock:
            proc = self._proc
            if proc is None or proc.stdin is None:
                return
            if self._access_unit_callback is not None:
                if source_stamp_ns is None or source_stamp_ns <= 0:
                    raise ValueError("H264 preview requires the source image timestamp")
                with self._metadata_lock:
                    if len(self._source_stamps) >= 64:
                        raise RuntimeError("H264 encoder output stalled")
                    self._source_stamps.append(source_stamp_ns)
            try:
                remaining = memoryview(frame)
                while remaining:
                    written = proc.stdin.write(remaining)
                    if written is None or written <= 0:
                        raise BrokenPipeError("encoder input pipe stopped accepting data")
                    remaining = remaining[written:]
                proc.stdin.flush()
            except (BrokenPipeError, OSError):
                self._restart_locked()

    def request_keyframe(self) -> None:
        # Stdin-driven command-line encoders expose no portable live force-IDR
        # event. Profiles configure a one-second GOP and repeated SPS/PPS, so
        # the next decoder-safe keyframe is bounded without replacing RTP SSRC.
        return

    def validate_runtime(self) -> None:
        raise NotImplementedError

    def _build_command(self) -> List[str]:
        raise NotImplementedError

    def _launch_locked(self) -> None:
        if not self._runtime_validated:
            self.validate_runtime()
            self._runtime_validated = True
        proc = subprocess.Popen(
            self._build_command(),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE if self._access_unit_callback else subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            bufsize=0,
        )
        with self._metadata_lock:
            self._source_stamps.clear()
            self._proc = proc
        self._stderr_tail.clear()
        if self._access_unit_callback:
            threading.Thread(target=self._drain_access_units, args=(proc,), daemon=True,
                             name="h264-preview-output").start()
        threading.Thread(target=self._drain_stderr, args=(proc,), daemon=True).start()

    def _restart_locked(self) -> None:
        proc = self._proc
        self._proc = None
        self._stop_process(proc, wait=False)
        self._launch_locked()

    @staticmethod
    def _stop_process(proc: Optional[subprocess.Popen], *, wait: bool = True) -> None:
        if proc is None:
            return
        try:
            if proc.stdin:
                proc.stdin.close()
        except OSError:
            pass
        if proc.poll() is None:
            try:
                proc.send_signal(signal.SIGTERM)
                if wait:
                    proc.wait(timeout=3)
                else:
                    # Restart is already handling a failed producer. Reap it
                    # promptly so bad input cannot accumulate old children.
                    proc.kill()
            except OSError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                except OSError:
                    pass
        try:
            proc.wait(timeout=1)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def _drain_access_units(self, proc: subprocess.Popen) -> None:
        parser = AnnexBAccessUnits()
        def emit(unit):
            with self._metadata_lock:
                if self._proc is not proc:
                    return
                if not self._source_stamps:
                    raise RuntimeError("H264 output has no matching source timestamp")
                stamp = self._source_stamps.popleft()
            self._access_unit_callback(unit, stamp)
        try:
            while self._proc is proc:
                data = proc.stdout.read(65536)
                if not data:
                    break
                for unit in parser.feed(data):
                    emit(unit)
            for unit in parser.finish():
                emit(unit)
        except Exception as exc:
            self._stderr_tail.append(f"H264 preview output failed: {exc}")
            if proc.poll() is None:
                proc.kill()

    def _drain_stderr(self, proc: subprocess.Popen) -> None:
        if proc.stderr is None:
            return
        try:
            for line in iter(proc.stderr.readline, b""):
                self._stderr_tail.append(line.decode("utf-8", errors="replace").rstrip())
        except Exception:
            pass


class FFmpegRtpEncoder(SubprocessRtpEncoder):
    """Portable FFmpeg backend; defaults to low-latency software x264."""

    def __init__(
        self,
        *,
        ffmpeg_path: str,
        rtp_host: str,
        rtp_port: int,
        width: int,
        height: int,
        fps: float,
        bitrate: int,
        input_format: str = "jpeg",
        encoder: str = "libx264",
        encoder_args: ArgumentInput = "[]",
        video_filter: str = "",
    ) -> None:
        super().__init__()
        self._ffmpeg_path = ffmpeg_path
        self._rtp_host = rtp_host
        self._rtp_port = int(rtp_port)
        self._width = int(width)
        self._height = int(height)
        self._fps = float(fps)
        self._bitrate = int(bitrate)
        self._input_format = normalize_input_format(input_format)
        self._encoder = encoder
        self._encoder_args = _parse_arguments(encoder_args, "ffmpeg_encoder_args_json")
        self._video_filter = video_filter

    def validate_runtime(self) -> None:
        try:
            result = subprocess.run(
                [
                    self._ffmpeg_path,
                    "-hide_banner",
                    "-loglevel",
                    "error",
                    "-h",
                    f"encoder={self._encoder}",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=False,
                timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"FFmpeg preflight failed: {exc}") from exc
        output = result.stdout.decode("utf-8", errors="replace")
        if result.returncode != 0 or "not recognized" in output.lower():
            raise RuntimeError(
                f"FFmpeg encoder {self._encoder!r} is unavailable: {output.strip()}"
            )

    def _build_command(self) -> List[str]:
        gop = max(1, int(round(self._fps)))
        video_filter = self._video_filter or (
            f"scale={self._width}:{self._height}:force_original_aspect_ratio=decrease,"
            f"pad={self._width}:{self._height}:(ow-iw)/2:(oh-ih)/2"
        )
        command = [
            self._ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            "error",
            "-fflags",
            "+genpts" if self._access_unit_callback else "nobuffer",
            "-flags",
            "low_delay",
        ]
        if self._access_unit_callback:
            command.extend(["-probesize", "32", "-analyzeduration", "0", "-threads", "1"])
        if self._input_format == "jpeg":
            # image2pipe + mjpeg accepts concatenated complete JPEG frames.
            command.extend(
                [
                    "-f",
                    "image2pipe",
                    "-vcodec",
                    "mjpeg",
                    "-framerate",
                    str(self._fps),
                    "-i",
                    "pipe:0",
                ]
            )
        else:
            pixel_format = _RAW_INPUTS[self._input_format][0]
            command.extend(
                [
                    "-f",
                    "rawvideo",
                    "-pixel_format",
                    pixel_format,
                    "-video_size",
                    f"{self._width}x{self._height}",
                    "-framerate",
                    str(self._fps),
                    "-i",
                    "pipe:0",
                ]
            )
        command.extend(["-an", "-vf", video_filter, "-c:v", self._encoder])

        if self._encoder_args:
            command.extend(self._expand_arguments(self._encoder_args, gop))
        elif self._encoder == "libx264":
            command.extend(
                [
                    "-preset",
                    "veryfast",
                    "-tune",
                    "zerolatency",
                    "-pix_fmt",
                    "yuv420p",
                    "-g",
                    str(gop),
                    "-keyint_min",
                    str(gop),
                    "-bf",
                    "0",
                    "-b:v",
                    str(self._bitrate),
                    "-maxrate",
                    str(self._bitrate),
                    "-bufsize",
                    str(self._bitrate * 2),
                    "-x264-params",
                    "repeat-headers=1:scenecut=0",
                ]
            )
        elif self._encoder == "h264_nvenc":
            # Bound the access-unit burst before sizing the loopback RTP
            # receive queue. A nominal average bitrate alone is not a bound:
            # motion or a scene cut can otherwise create an arbitrarily larger
            # short burst even while a static camera appears to sustain 30 Hz.
            command.extend(
                [
                    "-preset",
                    "llhq",
                    "-profile:v",
                    "high",
                    "-pix_fmt",
                    "yuv420p",
                    "-rc",
                    "cbr_ld_hq",
                    "-zerolatency",
                    "1",
                    "-delay",
                    # MJPEG CPU decode and NVENC can overlap across bounded surfaces.
                    # Zero serializes both stages on each frame (4K falls below 30 Hz).
                    "2" if self._input_format == "jpeg" else "0",
                    "-rc-lookahead",
                    "0",
                    "-bf",
                    "0",
                    "-g",
                    str(gop),
                    "-keyint_min",
                    str(gop),
                    "-no-scenecut",
                    "1",
                    "-strict_gop",
                    "1",
                    "-forced-idr",
                    "1",
                    "-b:v",
                    str(self._bitrate),
                    "-maxrate",
                    str(self._bitrate),
                    "-bufsize",
                    str(self._bitrate),
                ]
            )
        else:
            # Minimal codec-level defaults. Hardware-specific flags belong in
            # ffmpeg_encoder_args_json so no vendor assumptions leak here.
            command.extend(["-b:v", str(self._bitrate), "-g", str(gop)])

        if self._access_unit_callback:
            command.extend([
                "-xerror", "-vsync", "0", "-bf", "0", "-map", "0:v:0",
                "-bsf:v", "dump_extra=freq=keyframe,h264_metadata=aud=insert",
                "-f", "tee",
                f"[f=rtp:payload_type=96]rtp://{self._rtp_host}:{self._rtp_port}?pkt_size=1200|[f=h264:flush_packets=1]pipe:1",
            ])
        else:
            command.extend([
                "-f", "rtp", "-payload_type", "96",
                f"rtp://{self._rtp_host}:{self._rtp_port}?pkt_size=1200",
            ])
        return command

    def _expand_arguments(self, arguments: Sequence[str], gop: int) -> List[str]:
        context = {
            "@bitrate": str(self._bitrate),
            "@bitrate_kbps": str(max(1, int(round(self._bitrate / 1000.0)))),
            "@fps": _format_number(self._fps),
            "@gop": str(gop),
            "@width": str(self._width),
            "@height": str(self._height),
        }
        return [context.get(argument, argument) for argument in arguments]


class GStreamerRtpEncoder(SubprocessRtpEncoder):
    """Configurable GStreamer backend for software or hardware elements."""

    def __init__(
        self,
        *,
        gstreamer_path: str,
        gstreamer_inspect_path: str,
        rtp_host: str,
        rtp_port: int,
        width: int,
        height: int,
        fps: float,
        bitrate: int,
        input_format: str = "jpeg",
        jpeg_parser: str = "jpegparse",
        jpeg_caps: str = "image/jpeg,framerate=@fps_fraction",
        jpeg_decoder: str = "jpegdec",
        video_converter: str = "videoconvert",
        video_scaler: str = "videoscale",
        raw_caps: str = (
            "video/x-raw,format=I420,width=@width,height=@height,framerate=@fps_fraction"
        ),
        h264_encoder: str = "x264enc",
        decoder_properties: PropertyInput = "{}",
        converter_properties: PropertyInput = "{}",
        encoder_properties: PropertyInput = (
            '{"bitrate":"@bitrate_kbps","byte-stream":true,'
            '"key-int-max":"@gop","speed-preset":"ultrafast",'
            '"tune":"zerolatency"}'
        ),
    ) -> None:
        super().__init__()
        self._gstreamer_path = gstreamer_path
        self._gstreamer_inspect_path = gstreamer_inspect_path
        self._rtp_host = rtp_host
        self._rtp_port = int(rtp_port)
        self._width = int(width)
        self._height = int(height)
        self._fps = float(fps)
        self._bitrate = int(bitrate)
        self._input_format = normalize_input_format(input_format)
        self._jpeg_parser = _validate_element_name(jpeg_parser, "gstreamer_jpeg_parser")
        self._jpeg_caps = self._validate_caps(
            jpeg_caps, "image/jpeg", "gstreamer_jpeg_caps"
        )
        self._jpeg_decoder = _validate_element_name(jpeg_decoder, "gstreamer_jpeg_decoder")
        self._video_converter = _validate_element_name(
            video_converter, "gstreamer_video_converter"
        )
        self._video_scaler = _validate_element_name(
            video_scaler, "gstreamer_video_scaler"
        )
        self._h264_encoder = _validate_element_name(
            h264_encoder, "gstreamer_h264_encoder"
        )
        self._raw_caps = self._validate_caps(
            raw_caps, "video/x-raw", "gstreamer_raw_caps"
        )
        self._decoder_properties = _parse_properties(
            decoder_properties, "gstreamer_decoder_properties_json"
        )
        self._converter_properties = _parse_properties(
            converter_properties, "gstreamer_converter_properties_json"
        )
        self._encoder_properties = _parse_properties(
            encoder_properties, "gstreamer_encoder_properties_json"
        )

    def validate_runtime(self) -> None:
        try:
            launch_result = subprocess.run(
                [self._gstreamer_path, "--version"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                check=False,
                timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise RuntimeError(f"GStreamer launcher preflight failed: {exc}") from exc
        if launch_result.returncode != 0:
            detail = launch_result.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(
                "GStreamer launcher preflight failed"
                + (f": {detail}" if detail else "")
            )

        elements = ["fdsrc"]
        if self._input_format == "jpeg":
            elements.extend([self._jpeg_parser, self._jpeg_decoder])
        else:
            elements.append("rawvideoparse")
        elements.extend(
            [
                self._video_converter,
                self._video_scaler,
                self._h264_encoder,
                "h264parse",
                "rtph264pay",
                "udpsink",
            ]
        )
        inspected: Dict[str, str] = {}
        for element in dict.fromkeys(elements):
            try:
                result = subprocess.run(
                    [self._gstreamer_inspect_path, element],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    check=False,
                    timeout=10,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                raise RuntimeError(f"GStreamer preflight failed for {element}: {exc}") from exc
            if result.returncode != 0:
                detail = result.stdout.decode("utf-8", errors="replace").strip()
                raise RuntimeError(
                    f"GStreamer element {element!r} is unavailable"
                    + (f": {detail}" if detail else "")
                )
            inspected[element] = result.stdout.decode("utf-8", errors="replace")

        property_sets = (
            (self._jpeg_decoder, self._decoder_properties)
            if self._input_format == "jpeg"
            else (None, {})
        ), (self._video_converter, self._converter_properties), (
            self._h264_encoder,
            self._encoder_properties,
        )
        for element, properties in property_sets:
            if element is None:
                continue
            output = inspected[element]
            for property_name in properties:
                pattern = re.compile(
                    rf"(?m)^\s+{re.escape(property_name)}\s*:"
                )
                if not pattern.search(output):
                    raise RuntimeError(
                        f"GStreamer element {element!r} has no configured "
                        f"property {property_name!r}"
                    )

    def _build_command(self) -> List[str]:
        command = [
            self._gstreamer_path,
            "-q",
            "fdsrc",
            "fd=0",
            "do-timestamp=true",
        ]
        if self._input_format == "jpeg":
            command.extend(
                [
                    "!",
                    self._expanded_caps(self._jpeg_caps),
                    "!",
                    self._jpeg_parser,
                    "!",
                    self._jpeg_decoder,
                ]
            )
            command.extend(self._property_arguments(self._decoder_properties))
        else:
            raw_format = _RAW_INPUTS[self._input_format][1]
            command.extend(
                [
                    "blocksize=%d"
                    % packed_frame_bytes(
                        self._input_format,
                        self._width,
                        self._height,
                    ),
                    "!",
                    "rawvideoparse",
                    f"format={raw_format}",
                    f"width={self._width}",
                    f"height={self._height}",
                    f"framerate={self._expanded_fps_fraction()}",
                ]
            )
        command.extend(["!", self._video_converter])
        command.extend(self._property_arguments(self._converter_properties))
        command.extend(
            [
                "!",
                self._video_scaler,
                "!",
                self._expanded_caps(self._raw_caps),
                "!",
                self._h264_encoder,
            ]
        )
        command.extend(self._property_arguments(self._encoder_properties))
        command.extend(
            [
                "!",
                "h264parse",
                "config-interval=-1",
                "!",
                "video/x-h264,stream-format=byte-stream,alignment=au",
                "!",
                "rtph264pay",
                "pt=96",
                "mtu=1200",
                "config-interval=-1",
                "!",
                "udpsink",
                f"host={self._rtp_host}",
                f"port={self._rtp_port}",
                "sync=false",
                "async=false",
            ]
        )
        return command

    @staticmethod
    def _validate_caps(value: str, media_type: str, parameter_name: str) -> str:
        if not value.startswith(media_type):
            raise ValueError(f"{parameter_name} must start with {media_type}")
        if "!" in value or "\x00" in value or "\n" in value or "\r" in value:
            raise ValueError(f"{parameter_name} contains a forbidden pipeline separator")
        return value

    def _expanded_caps(self, template: str) -> str:
        replacements = {
            "@width": str(self._width),
            "@height": str(self._height),
            "@fps_fraction": self._expanded_fps_fraction(),
            "@fps": _format_number(self._fps),
        }
        caps = template
        for marker, replacement in replacements.items():
            caps = caps.replace(marker, replacement)
        return caps

    def _expanded_fps_fraction(self) -> str:
        fps_fraction = Fraction(str(self._fps)).limit_denominator(1001)
        return f"{fps_fraction.numerator}/{fps_fraction.denominator}"

    def _property_arguments(self, properties: Mapping[str, PropertyValue]) -> List[str]:
        gop = max(1, int(round(self._fps)))
        context: Dict[str, PropertyValue] = {
            "@bitrate": self._bitrate,
            "@bitrate_kbps": max(1, int(round(self._bitrate / 1000.0))),
            "@fps": self._fps,
            "@gop": gop,
            "@width": self._width,
            "@height": self._height,
        }
        arguments = []
        for key, value in properties.items():
            expanded = context.get(value, value) if isinstance(value, str) else value
            if isinstance(expanded, bool):
                rendered = "true" if expanded else "false"
            elif isinstance(expanded, float):
                rendered = _format_number(expanded)
            else:
                rendered = str(expanded)
            arguments.append(f"{key}={rendered}")
        return arguments


def create_rtp_encoder(*, backend: str, **kwargs: Any) -> SubprocessRtpEncoder:
    """Create a backend without auto-detecting a vendor or device model."""

    normalized = backend.strip().lower()
    if normalized == "ffmpeg":
        return FFmpegRtpEncoder(**kwargs)
    if normalized == "gstreamer":
        return GStreamerRtpEncoder(**kwargs)
    raise ValueError("encoder_backend must be one of: ffmpeg, gstreamer")
