"""Bounded framing of encoder Annex-B output with explicitly inserted AUDs."""
import re

_AUD = re.compile(b"\x00\x00(?:\x00)?\x01\x09")

class AnnexBAccessUnits:
    def __init__(self, maximum_bytes=8 * 1024 * 1024):
        self._buffer = bytearray()
        self._maximum = maximum_bytes

    def feed(self, data):
        self._buffer.extend(data)
        units = []
        boundaries = list(_AUD.finditer(self._buffer))
        if len(boundaries) > 1:
            start = 0
            for match in boundaries[1:]:
                end = match.start()
                if end - start > self._maximum:
                    raise ValueError("H264 access unit exceeds bounded preview buffer")
                units.append(bytes(self._buffer[start:end]))
                start = end
            del self._buffer[:start]
        if len(self._buffer) > self._maximum:
            raise ValueError("H264 access unit exceeds bounded preview buffer")
        return units

    def finish(self):
        data = bytes(self._buffer)
        self._buffer.clear()
        return [data] if data and _AUD.search(data) else []
