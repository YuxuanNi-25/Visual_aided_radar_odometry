import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='doppler', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='rcs', offset=16, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 20

def make_pc2(frame_id: str, stamp_msg, pts5: np.ndarray) -> PointCloud2:
    if pts5.dtype != np.float32:
        pts5 = pts5.astype(np.float32)
    msg = PointCloud2()
    msg.header = Header(frame_id=frame_id, stamp=stamp_msg)
    msg.height = 1
    msg.width = int(pts5.shape[0])
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = False
    msg.data = pts5.tobytes(order='C')
    return msg
