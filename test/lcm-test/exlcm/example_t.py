"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class example_t(object):
    __slots__ = ["timestamp", "position", "orientation", "num_ranges", "ranges", "name", "enabled"]

    def __init__(self):
        self.timestamp = 0
        self.position = [ 0.0 for dim0 in range(3) ]
        self.orientation = [ 0.0 for dim0 in range(4) ]
        self.num_ranges = 0
        self.ranges = []
        self.name = ""
        self.enabled = False

    def encode(self):
        buf = BytesIO()
        buf.write(example_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">q", self.timestamp))
        buf.write(struct.pack('>3d', *self.position[:3]))
        buf.write(struct.pack('>4d', *self.orientation[:4]))
        buf.write(struct.pack(">i", self.num_ranges))
        buf.write(struct.pack('>%dh' % self.num_ranges, *self.ranges[:self.num_ranges]))
        __name_encoded = self.name.encode('utf-8')
        buf.write(struct.pack('>I', len(__name_encoded)+1))
        buf.write(__name_encoded)
        buf.write(b"\0")
        buf.write(struct.pack(">b", self.enabled))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != example_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return example_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = example_t()
        self.timestamp = struct.unpack(">q", buf.read(8))[0]
        self.position = struct.unpack('>3d', buf.read(24))
        self.orientation = struct.unpack('>4d', buf.read(32))
        self.num_ranges = struct.unpack(">i", buf.read(4))[0]
        self.ranges = struct.unpack('>%dh' % self.num_ranges, buf.read(self.num_ranges * 2))
        __name_len = struct.unpack('>I', buf.read(4))[0]
        self.name = buf.read(__name_len)[:-1].decode('utf-8', 'replace')
        self.enabled = bool(struct.unpack('b', buf.read(1))[0])
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if example_t in parents: return 0
        tmphash = (0x1baa9e29b0fbaa8b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if example_t._packed_fingerprint is None:
            example_t._packed_fingerprint = struct.pack(">Q", example_t._get_hash_recursive([]))
        return example_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

