"""Exercise the real codec: frame identity, complete AUs and decoder late join."""
from io import BytesIO
import re
import os
import shutil
import subprocess
import threading

import pytest

from ros_image_rtp_adapter.encoder import FFmpegRtpEncoder


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="FFmpeg is not installed")
@pytest.mark.parametrize("encoder_name", ["libx264", pytest.param("h264_nvenc", marks=pytest.mark.skipif(
    os.environ.get("XGC2_TEST_NVENC") != "1", reason="NVENC hardware test is opt-in"))])
@pytest.mark.parametrize("input_format", ["jpeg", "rgb8"])
def test_h264_source_timestamps_match_decoded_pixels_and_late_join(input_format, encoder_name):
    Image = pytest.importorskip("PIL.Image")
    units = []
    complete = threading.Event()
    # NVENC also exercises this fixture; tiny 128x72 surfaces are unsupported.
    count, width, height = 36, 256, 144

    def receive(data, stamp):
        units.append((data, stamp))
        if len(units) == count:
            complete.set()

    encoder = FFmpegRtpEncoder(
        ffmpeg_path="ffmpeg", rtp_host="127.0.0.1", rtp_port=59994,
        width=width, height=height, fps=15, bitrate=1_000_000,
        input_format=input_format, encoder=encoder_name,
    )
    encoder.set_access_unit_callback(receive)
    encoder.start()
    try:
        for index in range(count):
            frame = Image.new("RGB", (width, height), (30 + index * 5,) * 3)
            if input_format == "jpeg":
                buffer = BytesIO()
                frame.save(buffer, format="JPEG")
                data = buffer.getvalue()
            else:
                data = frame.tobytes()
            encoder.write_frame(data, 1_000_000_000 + index * 66_666_667)
        # Finite input flushes the final AU; production frames delimit each other.
        encoder._proc.stdin.close()
        assert complete.wait(10), encoder.diagnostic
        assert encoder._proc.wait(timeout=5) == 0, encoder.diagnostic
        assert [stamp for _, stamp in units] == [
            1_000_000_000 + index * 66_666_667 for index in range(count)
        ]
        keyframes = []
        for index, (unit, _) in enumerate(units):
            nals = [n[0] & 31 for n in re.split(b"\x00\x00(?:\x00)?\x01", unit) if n]
            if 5 in nals:
                assert 7 in nals and 8 in nals
                keyframes.append(index)
        assert len(keyframes) >= 2
        for start in (0, keyframes[1]):
            decoded = subprocess.run([
                "ffmpeg", "-hide_banner", "-loglevel", "error", "-xerror",
                "-f", "h264", "-i", "pipe:0", "-f", "rawvideo",
                "-pix_fmt", "rgb24", "-vsync", "0", "pipe:1",
            ], input=b"".join(unit for unit, _ in units[start:]),
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True, timeout=10).stdout
            frame_bytes = width * height * 3
            assert len(decoded) == (count - start) * frame_bytes
            for offset in range(count - start):
                pixels = decoded[offset * frame_bytes:(offset + 1) * frame_bytes]
                assert abs(sum(pixels) / len(pixels) - (30 + (offset + start) * 5)) < 3
    finally:
        encoder.stop()
