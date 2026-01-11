#!/usr/bin/env python3
import os, sys, math, csv, time, threading, termios, fcntl
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf
import tf2_ros

from groundgrid.srv import NextCloud

DESIRED_RATE = 66  # Hz = 1/s


def float_to_time_msg(value: float) -> TimeMsg:
    sec = int(value)
    nanosec = int((value - sec) * 1e9)
    return TimeMsg(sec=sec, nanosec=nanosec)


class KittiDataPublisher(Node):
    def __init__(self):
        super().__init__('kitti_data_publisher')
        self.declare_parameter('/kitti_player/sequence', 0)
        self.declare_parameter('/kitti_player/directory', '')
        self.declare_parameter('/kitti_player/start', 0.0)
        self.declare_parameter('/kitti_player/end', float('inf'))
        self.declare_parameter('/kitti_player/paused', True)
        self.declare_parameter('rate', 0.6)

        sequence = f"{self.get_parameter('/kitti_player/sequence').get_parameter_value().integer_value:02d}"
        directory = self.get_parameter('/kitti_player/directory').get_parameter_value().string_value
        if directory == '':
            self.get_logger().error("no dataset directory given")
            raise SystemExit(1)
        if directory[-1] != '/':
            directory += '/'
        self.dir = os.path.join(directory, 'sequences', sequence, '')

        self.timestamps = pd.read_csv(f"{self.dir}times.txt", header=None, dtype=float).squeeze("columns")

        self.sim_clock = Clock()
        self.sim_clock.clock = float_to_time_msg(self.get_parameter('/kitti_player/start').get_parameter_value().double_value)
        self.sim_speed_multiplier = float(self.get_parameter('rate').get_parameter_value().double_value)
        self.paused = self.get_parameter('/kitti_player/paused').get_parameter_value().bool_value
        self.stepping = False
        self.end_timestamp = self.get_parameter('/kitti_player/end').get_parameter_value().double_value
        self.pausedTime = float_to_time_msg(0.0)

        self.cloudnum = 0
        self.currentstamp = -1
        self.time_start = time.time() - (self.sim_clock.clock.sec + self.sim_clock.clock.nanosec * 1e-9)

        self.processPoses()

        self.pubTime = self.create_publisher(Clock, 'clock', 10)
        self.pubCloud = self.create_publisher(PointCloud2, '/kitti/cloud', 10)
        self.pubPosition = self.create_publisher(Odometry, '/localization/odometry/filtered_map', 1)
        self.br = tf2_ros.TransformBroadcaster(self)

        self.create_service(NextCloud, '/kitti_player/NextCloud', self.step)

        self.timer = self.create_timer(1.0 / DESIRED_RATE, self.loop_once)
        self.get_logger().info('kitti_data_publisher initialized')

    def loop_once(self):
        start_loop = time.time()
        if not self.paused or self.stepping:
            new_time = self.sim_speed_multiplier * (time.time() - self.time_start) - (self.pausedTime.sec + self.pausedTime.nanosec * 1e-9)
            if new_time > self.end_timestamp:
                rclpy.shutdown()
                return
            self.sim_clock.clock = float_to_time_msg(new_time)
        else:
            paused_delta = self.sim_speed_multiplier * (time.time() - self.time_start)
            self.pausedTime = float_to_time_msg(paused_delta - (self.sim_clock.clock.sec + self.sim_clock.clock.nanosec * 1e-9))

        self.pubTime.publish(self.sim_clock)
        currentTime = self.sim_clock.clock
        stampnum, nextstamp = self.getNextTimeStamp(currentTime.sec + currentTime.nanosec * 1e-9)
        self.cloudnum = stampnum

        if nextstamp > self.currentstamp:
            self.sendPosition(currentTime)
            self.sendCloud(currentTime)
            self.get_logger().info(f"published pointcloud and position --> cloudnum={self.cloudnum} with timestamp {nextstamp}")
            self.currentstamp = nextstamp
            if self.stepping:
                self.stepping = False
                self.paused = True
        elif self.currentstamp > 0.0 and nextstamp == 0.0:
            rclpy.shutdown()

    def sendCloud(self, currentTime: TimeMsg):
        cloudnumstring = f'{self.cloudnum:06}.bin'
        scan = np.fromfile(self.dir + "velodyne/" + cloudnumstring, dtype=np.float32).reshape((-1, 4))
        labels = self.readLabels(self.dir, self.cloudnum)
        tst2 = np.hstack((scan, labels[:, None].astype(dtype=np.uint16)))

        sarray = np.core.records.fromarrays(tst2.T,
                                            dtype=[('x', 'float32'),
                                                   ('y', 'float32'),
                                                   ('z', 'float32'),
                                                   ('remission', 'float32'),
                                                   ('ring', 'uint16')])
        cloud_msg = PointCloud2()
        cloud_msg.header.stamp = currentTime
        cloud_msg.header.frame_id = 'kitti_base_link'
        cloud_msg.height = sarray.shape[0]
        cloud_msg.width = 1
        cloud_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1)
        ]
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 18
        cloud_msg.row_step = cloud_msg.point_step
        cloud_msg.is_dense = True
        cloud_msg.data = sarray.tobytes()
        self.pubCloud.publish(cloud_msg)

    def readLabels(self, dir_path, cloudnum):
        cloudnumstring = f'{cloudnum:06}.label'
        labels = np.fromfile(os.path.join(dir_path, "labels", cloudnumstring), dtype=np.uint32)
        labels = labels.reshape((-1))
        labels = labels & 0xFFFF
        return labels

    def processPoses(self):
        self.poses = []
        pose_file = os.path.join(self.dir, 'poses.txt')
        calibstring = "4.276802385584e-04 -9.999672484946e-01 -8.084491683471e-03 -1.198459927713e-02 -7.210626507497e-03 8.081198471645e-03 -9.999413164504e-01 -5.403984729748e-02 9.999738645903e-01 4.859485810390e-04 -7.206933692422e-03 -2.921968648686e-01"
        calib = np.fromstring(calibstring, dtype=float, sep=' ').reshape(3, 4)
        calib = np.vstack((calib, [0, 0, 0, 1]))
        calib_inv = np.linalg.inv(calib)
        with open(pose_file, 'r') as f:
            for line in f.readlines():
                pose = np.fromstring(line, dtype=float, sep=' ').reshape(3, 4)
                pose = np.vstack((pose, [0, 0, 0, 1]))
                self.poses.append(np.matmul(calib_inv, np.matmul(pose, calib)))

    def sendPosition(self, currentTime: TimeMsg):
        odom = Odometry()
        odom.header.stamp = currentTime
        odom.header.frame_id = "kitti_map"

        odom.pose.pose.position.x = float(self.poses[self.cloudnum][0][3])
        odom.pose.pose.position.y = float(self.poses[self.cloudnum][1][3])
        odom.pose.pose.position.z = float(self.poses[self.cloudnum][2][3])

        R = self.poses[self.cloudnum]
        q = tf.quaternion_from_matrix(R)
        q_rot = tf.quaternion_from_euler(0, 0, 0)
        q_new = tf.quaternion_multiply(q_rot, q)
        odom.pose.pose.orientation.x = q_new[0]
        odom.pose.pose.orientation.y = q_new[1]
        odom.pose.pose.orientation.z = q_new[2]
        odom.pose.pose.orientation.w = q_new[3]

        t = TransformStamped()
        t.header.stamp = currentTime
        t.header.frame_id = "map"
        t.child_frame_id = "kitti_base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation.x = odom.pose.pose.orientation.x
        t.transform.rotation.y = odom.pose.pose.orientation.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.br.sendTransform(t)

        self.pubPosition.publish(odom)

    def getNextTimeStamp(self, time_passed: float):
        timestamps_larger = self.timestamps[self.timestamps > time_passed]
        if timestamps_larger.empty:
            first_timestamp_larger_index = self.timestamps.shape[0]
        else:
            first_timestamp_larger_index = timestamps_larger.index[0]
        return (first_timestamp_larger_index - 1, self.timestamps.iloc[first_timestamp_larger_index - 1])

    def step(self, request, response):
        self.paused = False
        self.stepping = True
        return response


def main():
    rclpy.init()
    node = KittiDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
