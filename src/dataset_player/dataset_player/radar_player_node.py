from pathlib import Path
import csv

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

from .bin_reader import read_frame
from .pc2_builder import make_pc2

class RadarPlayer(Node):
    def __init__(self):
        super().__init__('radar_player')
        self.declare_parameters('', [
            ('radar_dir', ''),
            ('radar_csv', 'radar.csv'),
            ('topic_points', '/radar/points'),
            ('frame_id', 'radar'),
            ('stamp_policy', 'now'),
            ('publish_rate_hz', 100.0),
            ('loop', False),
            ('start_seq', 0),
            ('end_seq', -1),
            ('publish_clock', False),
        ])
        gp = lambda k: self.get_parameter(k).value
        self.radar_dir = Path(gp('radar_dir'))
        self.radar_csv_path = self.radar_dir / str(gp('radar_csv'))
        self.topic = str(gp('topic_points'))
        self.frame_id = str(gp('frame_id'))
        self.stamp_policy = str(gp('stamp_policy'))
        self.loop = bool(gp('loop'))
        self.start_seq = int(gp('start_seq'))
        self.end_seq = int(gp('end_seq'))
        self.publish_clock = bool(gp('publish_clock'))

        self.pub = self.create_publisher(PointCloud2, self.topic, 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10) if self.publish_clock else None

        self.index = self._load_index()
        self.i = 0
        rate = float(gp('publish_rate_hz'))
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._tick)

        self.get_logger().info(f"Loaded {len(self.index)} radar frames from {self.radar_csv_path}")
        self.get_logger().info(f"Publishing {self.topic} @ {rate} Hz | stamp_policy={self.stamp_policy} | loop={self.loop}")

    def _load_index(self):
        rows=[]
        with open(self.radar_csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for r in reader:
                seq = int(r['seq'])
                if seq < self.start_seq:
                    continue
                if self.end_seq >= 0 and seq > self.end_seq:
                    continue
                rows.append((int(r['t_ns']), seq, r['filename']))
        rows.sort(key=lambda x: x[1])
        return rows

    @staticmethod
    def _time_from_ns(t_ns:int) -> Time:
        sec = t_ns // 1_000_000_000
        nsec = t_ns % 1_000_000_000
        return Time(sec=int(sec), nanosec=int(nsec))

    def _tick(self):
        if self.i >= len(self.index):
            if self.loop:
                self.i = 0
            else:
                self.get_logger().info('Playback finished.')
                self.timer.cancel()
                return

        t_ns, seq, fname = self.index[self.i]
        self.i += 1

        frame = read_frame(self.radar_dir / fname)

        if self.stamp_policy == 'bag':
            stamp = self._time_from_ns(frame.t_ns)
        else:
            stamp = self.get_clock().now().to_msg()

        pc2 = make_pc2(self.frame_id, stamp, frame.points)
        self.pub.publish(pc2)

        if self.clock_pub is not None and self.stamp_policy == 'bag':
            ck = Clock()
            ck.clock = stamp
            self.clock_pub.publish(ck)

def main():
    rclpy.init()
    node = RadarPlayer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
