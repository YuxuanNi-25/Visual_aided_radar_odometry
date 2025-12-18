import struct
from pathlib import Path
import numpy as np

MAGIC = 0x46524452  # 'RDRF' as uint32 little-endian
_HDR_FMT = "<IHHQIIIIff8f"
_HDR_SIZE = 64

class RadarFrame:
    __slots__ = ("t_ns","seq","n_points","points")
    def __init__(self, t_ns:int, seq:int, points:np.ndarray):
        self.t_ns=t_ns
        self.seq=seq
        self.points=points
        self.n_points=int(points.shape[0])

def read_frame(bin_path: Path) -> RadarFrame:
    data = bin_path.read_bytes()
    if len(data) < _HDR_SIZE:
        raise ValueError(f"File too small: {bin_path}")
    header = struct.unpack(_HDR_FMT, data[:_HDR_SIZE])
    magic, version, header_bytes, t_ns, seq, n_points, point_format, flags, eps_xyz, rcs_min, *_ = header
    if magic != MAGIC:
        raise ValueError(f"Bad magic in {bin_path}: {hex(magic)}")
    if header_bytes != _HDR_SIZE:
        raise ValueError(f"Unexpected header size {header_bytes} in {bin_path}")
    if point_format != 1:
        raise ValueError(f"Unsupported point_format={point_format} in {bin_path}")
    pts = np.frombuffer(data[_HDR_SIZE:], dtype=np.float32)
    want = int(n_points) * 5
    if pts.size < want:
        # tolerate short files
        want = (pts.size // 5) * 5
    pts = pts[:want].reshape((-1,5))
    return RadarFrame(int(t_ns), int(seq), pts.copy())
