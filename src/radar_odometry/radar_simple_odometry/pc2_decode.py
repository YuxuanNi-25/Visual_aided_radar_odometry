import numpy as np
from sensor_msgs.msg import PointCloud2
import struct

def decode_xyz_doppler_rcs(msg: PointCloud2) -> np.ndarray:
    """Return Nx5 float32 array [x,y,z,doppler,rcs] from PointCloud2 with those fields."""
    # assume packed float32 in order; fallback by parsing offsets
    # We'll parse via field offsets to be safe.
    fields = {f.name: f for f in msg.fields}
    need = ['x','y','z','doppler','rcs']
    for n in need:
        if n not in fields:
            raise ValueError(f"PointCloud2 missing field {n}")
    step = msg.point_step
    buf = memoryview(msg.data)
    npts = msg.width * msg.height
    out = np.empty((npts,5), dtype=np.float32)
    for i in range(npts):
        base = i*step
        for j, n in enumerate(need):
            off = fields[n].offset
            out[i,j] = struct.unpack_from('<f', buf, base+off)[0]
    return out
