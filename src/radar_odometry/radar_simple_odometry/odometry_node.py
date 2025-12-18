import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_matrix

from sensor_msgs.msg import PointCloud2

from .pc2_decode import decode_xyz_doppler_rcs
from .icp import icp_point_to_point

def _apply_voxel(pts: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0.0 or pts.shape[0] == 0:
        return pts
    # simple voxel grid by rounding
    q = np.floor(pts / voxel).astype(np.int32)
    _, idx = np.unique(q, axis=0, return_index=True)
    return pts[idx]

class RadarOdometry(Node):
    def __init__(self):
        super().__init__('radar_odometry')
        self.declare_parameters('', [
            ('input_topic', '/radar/points'),
            ('frame_id_odom', 'odom'),
            ('frame_id_child', 'radar'),
            ('publish_tf', True),
            ('publish_path', True),
            ('max_icp_iters', 15),
            ('max_corr_dist', 1.5),
            ('min_points', 200),
            ('downsample_voxel', 0.0),
            ('dynamic_filter.enabled', True),
            ('dynamic_filter.doppler_abs_max', 8.0),
        ])
        gp=lambda k: self.get_parameter(k).value

        self.input_topic = str(gp('input_topic'))
        self.frame_odom = str(gp('frame_id_odom'))
        self.frame_child = str(gp('frame_id_child'))
        self.publish_tf = bool(gp('publish_tf'))
        self.publish_path = bool(gp('publish_path'))
        self.max_iters = int(gp('max_icp_iters'))
        self.max_corr = float(gp('max_corr_dist'))
        self.min_points = int(gp('min_points'))
        self.voxel = float(gp('downsample_voxel'))
        self.df_enabled = bool(gp('dynamic_filter.enabled'))
        self.dop_max = float(gp('dynamic_filter.doppler_abs_max'))

        self.sub = self.create_subscription(PointCloud2, self.input_topic, self._cb, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_path = self.create_publisher(Path, '/path', 10) if self.publish_path else None
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        self.T_odom_child = np.eye(4, dtype=np.float64)
        self.prev_xyz = None
        self.path = Path()
        self.path.header.frame_id = self.frame_odom

        self.get_logger().info(f"Subscribing {self.input_topic}")
        self.get_logger().info(f"Publishing /odom frame_id={self.frame_odom}, child={self.frame_child}")

    def _filter_dynamic(self, pts5: np.ndarray) -> np.ndarray:
        if not self.df_enabled:
            return pts5
        # minimal doppler gating
        mask = np.abs(pts5[:,3]) <= self.dop_max
        return pts5[mask]

    def _publish(self, stamp):
        # Odometry
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_odom
        odom.child_frame_id = self.frame_child

        T = self.T_odom_child
        q = quaternion_from_matrix(T)
        odom.pose.pose.position.x = float(T[0,3])
        odom.pose.pose.position.y = float(T[1,3])
        odom.pose.pose.position.z = float(T[2,3])
        odom.pose.pose.orientation.x = float(q[0])
        odom.pose.pose.orientation.y = float(q[1])
        odom.pose.pose.orientation.z = float(q[2])
        odom.pose.pose.orientation.w = float(q[3])
        self.pub_odom.publish(odom)

        if self.tf_br is not None:
            tfm = TransformStamped()
            tfm.header.stamp = stamp
            tfm.header.frame_id = self.frame_odom
            tfm.child_frame_id = self.frame_child
            tfm.transform.translation.x = float(T[0,3])
            tfm.transform.translation.y = float(T[1,3])
            tfm.transform.translation.z = float(T[2,3])
            tfm.transform.rotation.x = float(q[0])
            tfm.transform.rotation.y = float(q[1])
            tfm.transform.rotation.z = float(q[2])
            tfm.transform.rotation.w = float(q[3])
            self.tf_br.sendTransform(tfm)

        if self.pub_path is not None:
            ps = PoseStamped()
            ps.header.stamp = stamp
            ps.header.frame_id = self.frame_odom
            ps.pose = odom.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(ps)
            # keep path size manageable
            if len(self.path.poses) > 5000:
                self.path.poses = self.path.poses[-5000:]
            self.pub_path.publish(self.path)

    def _cb(self, msg: PointCloud2):
        try:
            pts5 = decode_xyz_doppler_rcs(msg)
        except Exception as e:
            self.get_logger().warn(f"decode failed: {e}")
            return
        pts5 = self._filter_dynamic(pts5)
        xyz = pts5[:, :3].astype(np.float64)

        xyz = _apply_voxel(xyz, self.voxel)

        if xyz.shape[0] < self.min_points:
            # not enough points, publish current pose (prediction-only)
            self._publish(msg.header.stamp)
            return

        if self.prev_xyz is None:
            self.prev_xyz = xyz
            self._publish(msg.header.stamp)
            return

        # Estimate relative transform: prev -> curr
        # We compute T such that curr ~= T * prev
        T_prev_to_curr = icp_point_to_point(self.prev_xyz, xyz, max_iters=self.max_iters, max_corr_dist=self.max_corr)

        # Integrate: odom->curr = odom->prev * (prev->curr)
        self.T_odom_child = self.T_odom_child @ T_prev_to_curr
        self.prev_xyz = xyz
        self._publish(msg.header.stamp)

def main():
    rclpy.init()
    node = RadarOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
