import pytest
from ros_image_rtp_adapter.h264 import AnnexBAccessUnits


def test_aud_framing_preserves_nals_and_split_start_codes():
    units = [b"\x00\x00\x00\x01\x09\xf0\x00\x00\x00\x01\x67sps\x00\x00\x01\x65idr",
             b"\x00\x00\x01\x09\xf0\x00\x00\x01\x41delta"]
    parser = AnnexBAccessUnits()
    result = []
    for value in b"".join(units):
        result += parser.feed(bytes([value]))
    result += parser.finish()
    assert result == units


def test_missing_boundary_cannot_grow_memory_without_bound():
    parser = AnnexBAccessUnits(maximum_bytes=32)
    with pytest.raises(ValueError):
        parser.feed(b"x" * 33)
