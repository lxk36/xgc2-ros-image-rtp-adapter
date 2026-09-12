from unittest.mock import Mock, patch

import pytest

from ros_image_rtp_adapter.encoder import (
    FFmpegRtpEncoder,
    GStreamerRtpEncoder,
    create_rtp_encoder,
)


def make_encoder(*, encoder="libx264"):
    return FFmpegRtpEncoder(
        ffmpeg_path="ffmpeg",
        rtp_host="127.0.0.1",
        rtp_port=5004,
        width=1280,
        height=720,
        fps=15.0,
        bitrate=2_500_000,
        encoder=encoder,
    )


def make_gstreamer_encoder(**overrides):
    values = {
        "gstreamer_path": "gst-launch-1.0",
        "gstreamer_inspect_path": "gst-inspect-1.0",
        "rtp_host": "127.0.0.1",
        "rtp_port": 5004,
        "width": 1280,
        "height": 720,
        "fps": 15.0,
        "bitrate": 2_500_000,
    }
    values.update(overrides)
    return GStreamerRtpEncoder(**values)


def test_soft_encoder_uses_one_second_repeated_header_gop():
    command = make_encoder()._build_command()

    assert command[command.index("-g") + 1] == "15"
    assert command[command.index("-keyint_min") + 1] == "15"
    assert command[command.index("-x264-params") + 1] == "repeat-headers=1:scenecut=0"


def test_keyframe_request_does_not_restart_live_ffmpeg_process():
    encoder = make_encoder()
    process = Mock()
    process.stdin = Mock()
    process.stdin.write.return_value = 4
    encoder._proc = process

    with patch("ros_image_rtp_adapter.encoder.subprocess.Popen") as popen:
        encoder.request_keyframe()
        encoder.write_frame(b"jpeg")

    popen.assert_not_called()
    process.stdin.write.assert_called_once_with(b"jpeg")
    process.stdin.flush.assert_called_once_with()


def test_non_x264_encoder_does_not_receive_x264_only_options():
    command = make_encoder(encoder="h264_nvenc")._build_command()

    assert "-x264-params" not in command


def test_nvenc_uses_a_bounded_low_latency_burst_contract():
    command = make_encoder(encoder="h264_nvenc")._build_command()

    assert command[command.index("-pix_fmt") + 1] == "yuv420p"
    assert command[command.index("-rc") + 1] == "cbr_ld_hq"
    assert command[command.index("-maxrate") + 1] == "2500000"
    assert command[command.index("-bufsize") + 1] == "2500000"
    assert command[command.index("-bf") + 1] == "0"
    assert command[command.index("-delay") + 1] == "2"
    assert command[command.index("-strict_gop") + 1] == "1"
    assert command[command.index("-no-scenecut") + 1] == "1"


def test_ffmpeg_raw_input_is_explicit_fixed_size_and_never_roundtrips_through_jpeg():
    command = FFmpegRtpEncoder(
        ffmpeg_path="ffmpeg",
        rtp_host="127.0.0.1",
        rtp_port=5004,
        width=640,
        height=360,
        fps=10.0,
        bitrate=1_000_000,
        input_format="bgr8",
    )._build_command()

    assert command[command.index("-f") + 1] == "rawvideo"
    assert command[command.index("-pixel_format") + 1] == "bgr24"
    assert command[command.index("-video_size") + 1] == "640x360"
    assert "mjpeg" not in command


def test_ffmpeg_custom_arguments_expand_backend_neutral_runtime_markers():
    encoder = FFmpegRtpEncoder(
        ffmpeg_path="ffmpeg",
        rtp_host="127.0.0.1",
        rtp_port=5004,
        width=640,
        height=360,
        fps=12.5,
        bitrate=3_000_000,
        encoder="some_h264_encoder",
        encoder_args='["-b:v","@bitrate","-g","@gop","-profile:v","main"]',
    )

    command = encoder._build_command()

    assert command[command.index("-b:v") + 1] == "3000000"
    assert command[command.index("-g") + 1] == "12"
    assert command[command.index("-profile:v") + 1] == "main"
    assert "-preset" not in command


def test_gstreamer_default_is_a_software_pipeline_with_expanded_caps():
    command = make_gstreamer_encoder()._build_command()

    assert command[0] == "gst-launch-1.0"
    assert "jpegdec" in command
    assert "videoconvert" in command
    assert "videoscale" in command
    assert "x264enc" in command
    assert "image/jpeg,framerate=15/1" in command
    assert (
        "video/x-raw,format=I420,width=1280,height=720,framerate=15/1" in command
    )
    assert "bitrate=2500" in command
    assert "key-int-max=15" in command
    assert "host=127.0.0.1" in command
    assert "port=5004" in command


def test_gstreamer_vendor_elements_are_profile_data_not_code_branches():
    command = make_gstreamer_encoder(
        jpeg_decoder="nvjpegdec",
        video_converter="nvvidconv",
        video_scaler="identity",
        raw_caps=(
            "video/x-raw(memory:NVMM),format=NV12,width=@width,"
            "height=@height,framerate=@fps_fraction"
        ),
        h264_encoder="nvv4l2h264enc",
        encoder_properties=(
            '{"control-rate":1,"bitrate":"@bitrate",'
            '"iframeinterval":"@gop","idrinterval":"@gop",'
            '"insert-sps-pps":true}'
        ),
    )._build_command()

    assert "nvjpegdec" in command
    assert "nvvidconv" in command
    assert "identity" in command
    assert "nvv4l2h264enc" in command
    assert "bitrate=2500000" in command
    assert "iframeinterval=15" in command
    assert "idrinterval=15" in command
    assert "insert-sps-pps=true" in command


def test_gstreamer_raw_input_uses_rawvideoparse_before_the_configured_converter():
    command = make_gstreamer_encoder(input_format="rgba8")._build_command()

    assert "rawvideoparse" in command
    assert "format=rgba" in command
    assert "width=1280" in command
    assert "height=720" in command
    assert command.index("rawvideoparse") < command.index("videoconvert")
    assert "jpegparse" not in command
    assert "jpegdec" not in command


def test_gstreamer_rejects_pipeline_injection_in_element_or_caps():
    with pytest.raises(ValueError, match="element factory"):
        make_gstreamer_encoder(h264_encoder="x264enc ! fakesink")
    with pytest.raises(ValueError, match="pipeline separator"):
        make_gstreamer_encoder(raw_caps="video/x-raw ! fakesink")


def test_json_configuration_rejects_non_scalar_gstreamer_properties():
    with pytest.raises(ValueError, match="string, number, or boolean"):
        make_gstreamer_encoder(encoder_properties='{"options":["unsafe"]}')


def test_backend_factory_never_auto_detects_hardware():
    encoder = create_rtp_encoder(
        backend="ffmpeg",
        ffmpeg_path="ffmpeg",
        rtp_host="127.0.0.1",
        rtp_port=5004,
        width=640,
        height=360,
        fps=10.0,
        bitrate=1_000_000,
    )
    assert isinstance(encoder, FFmpegRtpEncoder)

    with pytest.raises(ValueError, match="ffmpeg, gstreamer"):
        create_rtp_encoder(backend="auto")


def test_gstreamer_preflight_reports_the_missing_element():
    encoder = make_gstreamer_encoder()
    launcher = Mock(returncode=0, stderr=b"")
    unavailable = Mock(returncode=1, stderr=b"No such element")

    with patch(
        "ros_image_rtp_adapter.encoder.subprocess.run",
        side_effect=[launcher, unavailable],
    ):
        with pytest.raises(RuntimeError, match="fdsrc"):
            encoder.validate_runtime()


def test_gstreamer_preflight_rejects_unknown_configured_properties():
    encoder = make_gstreamer_encoder(encoder_properties='{"not-a-property":1}')
    launcher = Mock(returncode=0, stderr=b"")
    inspection = Mock(returncode=0, stdout=b"Element Properties:\n  name : name\n")

    with patch(
        "ros_image_rtp_adapter.encoder.subprocess.run",
        side_effect=[launcher] + [inspection] * 16,
    ):
        with pytest.raises(RuntimeError, match="not-a-property"):
            encoder.validate_runtime()


def test_encoder_finishes_short_pipe_writes_before_accepting_next_frame():
    encoder = make_encoder()
    process = Mock()
    process.stdin.write.side_effect = [2, 2]
    encoder._proc = process
    encoder.write_frame(b"jpeg")
    assert [bytes(call.args[0]) for call in process.stdin.write.call_args_list] == [b"jpeg", b"eg"]
    process.stdin.flush.assert_called_once_with()
