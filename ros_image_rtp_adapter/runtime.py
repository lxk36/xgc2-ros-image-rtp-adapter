"""ROS-neutral image adapter lifecycle shared by ROS 1 and ROS 2."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Callable, Deque, Dict, Optional, Tuple

from ros_image_rtp_adapter.control_socket import SourceControlServer, SourceDescription
from ros_image_rtp_adapter.encoder import (
    SubprocessRtpEncoder,
    create_rtp_encoder,
)
from ros_image_rtp_adapter.frames import (
    FrameValidationError,
    RawFrame,
    pack_raw_frame,
    require_jpeg_bytes,
)
from ros_image_rtp_adapter.settings import AdapterSettings


LogFunction = Callable[[str], None]
EncoderFactory = Callable[..., SubprocessRtpEncoder]


@dataclass(frozen=True)
class _QueuedFrame:
    encoder_data: bytes
    source_stamp_ns: Optional[int] = None
    jpeg_snapshot: Optional[bytes] = None
    raw_snapshot: Optional[RawFrame] = None


class ImageRtpAdapterRuntime:
    """Own one encoder, source-control socket, and bounded frame queue."""

    def __init__(
        self,
        settings: AdapterSettings,
        *,
        log_info: Optional[LogFunction] = None,
        log_warning: Optional[LogFunction] = None,
        log_error: Optional[LogFunction] = None,
        encoder_factory: EncoderFactory = create_rtp_encoder,
        on_access_unit: Optional[Callable[[bytes, int], None]] = None,
    ) -> None:
        self.settings = settings
        self._log_info = log_info or (lambda _message: None)
        self._log_warning = log_warning or (lambda _message: None)
        self._log_error = log_error or (lambda _message: None)
        self._encoder = encoder_factory(
            backend=settings.encoder_backend,
            **settings.encoder_kwargs(),
        )
        self._on_access_unit = on_access_unit
        if on_access_unit is not None:
            self._encoder.set_access_unit_callback(on_access_unit)
        self._consumer_lock = threading.Lock()
        self._edge_active = False
        self._video_active = False
        self._lock = threading.Lock()
        self._frame_condition = threading.Condition(self._lock)
        self._encoder_lock = threading.Lock()
        self._pending: Deque[_QueuedFrame] = deque(
            maxlen=1 if settings.drop_to_latest else 32
        )
        self._latest: Optional[_QueuedFrame] = None
        self._active = False
        self._started = False
        self._pump_stop = threading.Event()
        self._pump_thread: Optional[threading.Thread] = None
        self._frames_in = 0
        self._frames_out = 0
        self._frames_dropped = 0
        self._last_validation_warning = 0.0
        description = SourceDescription(
            source_id=settings.source_id,
            rtp_host=settings.rtp_host,
            rtp_port=settings.rtp_port,
            width=settings.width,
            height=settings.height,
            fps=settings.fps,
            frame_id=settings.frame_id,
            snapshot_jpeg_backend=(
                "source-jpeg-passthrough"
                if settings.input_message_type == "compressed"
                else "pillow-libjpeg"
            ),
        )
        self._control = SourceControlServer(
            settings.control_socket,
            description,
            on_set_active=self.set_active,
            on_request_keyframe=self._encoder.request_keyframe,
            on_snapshot=self.snapshot_parts,
            snapshot_readback=(
                "fresh-compressed-frame"
                if settings.input_message_type == "compressed"
                else "fresh-raw-frame"
            ),
        )

    @property
    def encoder(self) -> SubprocessRtpEncoder:
        return self._encoder

    def start(self) -> None:
        if self._started:
            return
        # Fail Session readiness immediately for a missing binary, element, or
        # configured property, while leaving the actual encoder unallocated
        # until Edge supplies the first consumer.
        self._encoder.preflight()
        self._pump_stop.clear()
        thread = threading.Thread(
            target=self._run_encoder_pump,
            name="image-rtp-encoder-pump",
            daemon=True,
        )
        self._pump_thread = thread
        self._started = True
        try:
            thread.start()
            self._control.start()
        except Exception:
            self._started = False
            self._pump_stop.set()
            with self._frame_condition:
                self._frame_condition.notify_all()
            if thread.is_alive():
                thread.join(timeout=2.0)
            self._pump_thread = None
            raise

    def stop(self) -> None:
        if not self._started:
            return
        self._started = False
        self._pump_stop.set()
        with self._frame_condition:
            self._frame_condition.notify_all()
        try:
            self._control.stop()
        finally:
            thread = self._pump_thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=5.0)
            self._pump_thread = None
            with self._encoder_lock:
                with self._lock:
                    self._active = False
                    self._pending.clear()
                self._encoder.stop()

    def set_active(self, active: bool) -> None:
        with self._consumer_lock:
            self._edge_active = bool(active)
            self._set_encoder_active(self._edge_active or self._video_active)

    def set_video_active(self, active: bool) -> None:
        with self._consumer_lock:
            self._video_active = bool(active)
            self._set_encoder_active(self._edge_active or self._video_active)

    def _set_encoder_active(self, active: bool) -> None:
        desired = bool(active)
        with self._encoder_lock:
            with self._lock:
                if desired == self._active:
                    return
                self._active = desired
                if not desired:
                    self._pending.clear()
            try:
                if desired:
                    self._encoder.start()
                    with self._frame_condition:
                        self._frame_condition.notify_all()
                else:
                    self._encoder.stop()
            except Exception:
                with self._lock:
                    self._active = False
                    self._pending.clear()
                self._encoder.stop()
                raise
        self._log_info(f"set-active -> {desired}")

    def submit_compressed(self, data: bytes, image_format: str, *, source_stamp_ns: Optional[int] = None) -> bool:
        if self.settings.input_message_type != "compressed":
            self._warn_validation("received CompressedImage while input_message_type=raw")
            return False
        normalized_format = (image_format or "").strip().lower()
        if (
            self.settings.require_jpeg
            and "jpeg" not in normalized_format
            and "jpg" not in normalized_format
        ):
            self._warn_validation(
                f"CompressedImage format {image_format!r} is not JPEG"
            )
            return False
        try:
            frame = (
                require_jpeg_bytes(bytes(data))
                if self.settings.require_jpeg
                else bytes(data)
            )
        except FrameValidationError as exc:
            self._warn_validation(str(exc))
            return False
        if not frame:
            return False
        return self._enqueue(_QueuedFrame(encoder_data=frame, jpeg_snapshot=frame, source_stamp_ns=source_stamp_ns))

    def submit_raw(
        self,
        data: bytes,
        *,
        width: int,
        height: int,
        step: int,
        encoding: str,
        source_stamp_ns: Optional[int] = None,
    ) -> bool:
        if self.settings.input_message_type != "raw":
            self._warn_validation("received Image while input_message_type=compressed")
            return False
        try:
            raw = pack_raw_frame(
                bytes(data),
                width=width,
                height=height,
                step=step,
                encoding=encoding,
                expected_width=self.settings.width,
                expected_height=self.settings.height,
                expected_encoding=self.settings.raw_encoding,
            )
        except FrameValidationError as exc:
            self._warn_validation(str(exc))
            return False
        return self._enqueue(_QueuedFrame(encoder_data=raw.data, raw_snapshot=raw, source_stamp_ns=source_stamp_ns))

    def _enqueue(self, frame: _QueuedFrame) -> bool:
        if self._on_access_unit and (frame.source_stamp_ns is None or frame.source_stamp_ns <= 0):
            self._warn_validation("H264 preview requires a valid source image timestamp")
            return False
        with self._frame_condition:
            self._latest = frame
            self._frames_in += 1
            self._frame_condition.notify_all()
            if not self._active:
                return True
            if len(self._pending) == self._pending.maxlen:
                self._frames_dropped += 1
            self._pending.append(frame)
        return True

    def pump(self) -> bool:
        with self._encoder_lock:
            with self._lock:
                if not self._active or not self._pending:
                    return False
                frame = self._pending.popleft()
            if self._on_access_unit:
                self._encoder.write_frame(frame.encoder_data, frame.source_stamp_ns)
            else:
                self._encoder.write_frame(frame.encoder_data)
        with self._lock:
            self._frames_out += 1
        return True

    def _run_encoder_pump(self) -> None:
        while not self._pump_stop.is_set():
            with self._frame_condition:
                self._frame_condition.wait_for(
                    lambda: self._pump_stop.is_set()
                    or (self._active and bool(self._pending))
                )
                if self._pump_stop.is_set():
                    return
            try:
                self.pump()
            except Exception as exc:
                self._log_error(f"encoder frame pump failed: {exc}")
                with self._encoder_lock:
                    with self._lock:
                        self._active = False
                        self._pending.clear()
                    self._encoder.stop()

    def snapshot_jpeg(self) -> Optional[bytes]:
        jpeg, _rgb = self.snapshot_parts(False) or (None, None)
        return jpeg

    def snapshot_parts(
        self,
        include_rgb: bool = True,
        require_fresh: bool = False,
    ) -> Optional[Tuple[bytes, bytes]]:
        with self._frame_condition:
            if require_fresh:
                request_frame = self._frames_in
                deadline = time.monotonic() + max(
                    0.25,
                    min(2.0, 3.0 / float(self.settings.fps)),
                )
                while self._frames_in <= request_frame:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        return None
                    self._frame_condition.wait(remaining)
            frame = self._latest
        if frame is None:
            return None
        jpeg: Optional[bytes] = None
        rgb = b""
        if frame.jpeg_snapshot is not None:
            jpeg = frame.jpeg_snapshot
        elif frame.raw_snapshot is not None:
            try:
                jpeg = frame.raw_snapshot.to_jpeg()
            except Exception as exc:
                self._log_error(f"raw snapshot JPEG conversion failed: {exc}")
        if include_rgb and frame.raw_snapshot is not None:
            try:
                rgb = frame.raw_snapshot.to_rgb()
            except Exception as exc:
                self._log_error(f"raw snapshot RGB conversion failed: {exc}")
        if include_rgb and jpeg and not rgb:
            try:
                from io import BytesIO
                from PIL import Image
                rgb = Image.open(BytesIO(jpeg)).convert("RGB").tobytes()
            except Exception as exc:
                self._log_error(f"JPEG snapshot RGB conversion failed: {exc}")
        if not jpeg:
            return None
        return jpeg, rgb

    def status(self) -> Dict[str, object]:
        with self._lock:
            return {
                "frames_in": self._frames_in,
                "frames_out": self._frames_out,
                "frames_dropped": self._frames_dropped,
                "active": self._active,
                "pending": len(self._pending),
                "encoder_running": self._encoder.running,
                "encoder_diagnostic": self._encoder.diagnostic,
            }

    def _warn_validation(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_validation_warning >= 5.0:
            self._last_validation_warning = now
            self._log_warning(message)
