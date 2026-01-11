#!/usr/bin/env python3
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from groundgrid.srv import NextCloud

class EvalGroundpointClassifier(Node):
    def __init__(self):
        super().__init__('eval_groundpoint_classifier')
        self.declare_parameter('kitti_player.sequence', '00')

        cfg_path = os.path.join(get_package_share_directory('groundgrid'), 'config', 'semantic-kitti-all.yaml')
        try:
            self.CFG = yaml.safe_load(open(cfg_path, 'r'))
        except Exception as e:
            self.get_logger().error(f"Error opening SemanticKITTI yaml configuration file: {e}")
            self.CFG = {"labels": {}}

        self.nonGroundPointLabelCount = {label: 0 for label in self.CFG["labels"].values()}
        self.semanticCloudLabelCount = {label: 0 for label in self.CFG["labels"].values()}
        self.truePositiveCloudLabelCount = {label: 0 for label in self.CFG["labels"].values()}
        self.falsePositiveCloudLabelCount = {label: 0 for label in self.CFG["labels"].values()}
        self.cloudCount = 0
        self.groundLabels = ["road", "sidewalk", "parking", "lane-marking"]
        self.additionalGroundLabels = ["other-ground", "terrain"]
        self.nonGroundLabels = ["bicycle", "moving-bicyclist", "motorcycle", "moving-motorcyclist", "person", "moving-person", "traffic-sign", "car", "moving-car",
                                "motorcyclist", "bicyclist", "truck", "moving-truck", "building", "fence", "trunk", "pole", "bus", "on-rails", "other-vehicle", "other-structure",
                                "other-object", "moving-on-rails", "moving-bus", "moving-other-vehicle"]

        self.subscription = self.create_subscription(
            PointCloud2,
            '/groundgrid/segmented_cloud',
            self.callback_predicted_cloud,
            10
        )

        self.next_cloud_client = self.create_client(NextCloud, '/kitti_player/NextCloud')
        while not self.next_cloud_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kitti_player/NextCloud service...')
        self.call_next_cloud()

    def call_next_cloud(self):
        req = NextCloud.Request()
        self.next_cloud_client.call_async(req)

    def callback_predicted_cloud(self, cloud: PointCloud2):
        for p in pc2.read_points(cloud, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True):
            intensity = p[3]
            label = p[4]
            labelstring = self.CFG["labels"].get(label, str(label))

            if labelstring not in self.nonGroundPointLabelCount:
                self.nonGroundPointLabelCount[labelstring] = 0
                self.semanticCloudLabelCount[labelstring] = 0
                self.truePositiveCloudLabelCount[labelstring] = 0
                self.falsePositiveCloudLabelCount[labelstring] = 0

            if intensity == 99:  # predicted obstacle
                self.nonGroundPointLabelCount[labelstring] += 1
            elif intensity == 49:  # predicted ground
                if labelstring in self.groundLabels or labelstring in self.additionalGroundLabels:
                    self.truePositiveCloudLabelCount[labelstring] += 1
                else:
                    self.falsePositiveCloudLabelCount[labelstring] += 1

            self.semanticCloudLabelCount[labelstring] += 1

        self.cloudCount += 1
        if self.cloudCount % 500 == 0:
            self.print_statistics()
        self.call_next_cloud()

    def print_statistics(self):
        sequence = self.get_parameter('kitti_player.sequence').get_parameter_value().string_value
        self.get_logger().info(f"Received {self.cloudCount} point clouds. KITTI sequence {sequence}.")

        header = "label\t\t\tnonground %\tground %\tnonground\ttotal"
        self.get_logger().info(header)
        for label in self.CFG["labels"].values():
            nonGroundPoints = self.nonGroundPointLabelCount.get(label, 0)
            totalPoints = self.semanticCloudLabelCount.get(label, 0)
            if totalPoints == 0:
                continue
            padded = label
            if len(padded) < 8:
                padded += '\t'
            if len(padded) < 16:
                padded += '\t'
            line = f"{padded}\t{nonGroundPoints/totalPoints:2.2%}\t\t{1.0 - (nonGroundPoints/totalPoints):2.2%}\t\t{nonGroundPoints}\t\t{totalPoints}"
            self.get_logger().info(line)

        truePgroundsum = 0
        truePgroundsumadd = 0
        trueNgroundNoVeg = 0
        falsePgroundObssumNoVeg = 0
        falseNgroundNoVeg = 0
        gtgroundsum = 0
        gtgroundsumadd = 0
        gtnonGroundSumNoVeg = 0

        for label in self.groundLabels:
            truePgroundsum += self.truePositiveCloudLabelCount.get(label, 0)
            gtgroundsum += self.semanticCloudLabelCount.get(label, 0)
            falseNgroundNoVeg += self.nonGroundPointLabelCount.get(label, 0)

        truePgroundsumadd = truePgroundsum
        gtgroundsumadd = gtgroundsum
        for label in self.additionalGroundLabels:
            truePgroundsumadd += self.truePositiveCloudLabelCount.get(label, 0)
            gtgroundsumadd += self.semanticCloudLabelCount.get(label, 0)
            falseNgroundNoVeg += self.nonGroundPointLabelCount.get(label, 0)

        for label in self.nonGroundLabels:
            falsePgroundObssumNoVeg += self.falsePositiveCloudLabelCount.get(label, 0)
            gtnonGroundSumNoVeg += self.semanticCloudLabelCount.get(label, 0)
            trueNgroundNoVeg += self.nonGroundPointLabelCount.get(label, 0)

        truePositiveGround = truePgroundsumadd
        trueNegativeGround = trueNgroundNoVeg
        falsePositiveGround = falsePgroundObssumNoVeg
        falseNegativeGround = falseNgroundNoVeg

        precision = truePositiveGround / (falsePositiveGround + truePositiveGround) if (falsePositiveGround + truePositiveGround) else 0.0
        recall = truePositiveGround / (falseNegativeGround + truePositiveGround) if (falseNegativeGround + truePositiveGround) else 0.0
        f1 = (2 * truePositiveGround) / (2 * truePositiveGround + falsePositiveGround + falseNegativeGround) if (2 * truePositiveGround + falsePositiveGround + falseNegativeGround) else 0.0
        accuracy = (truePositiveGround + trueNegativeGround) / (truePositiveGround + trueNegativeGround + falsePositiveGround + falseNegativeGround) if (truePositiveGround + trueNegativeGround + falsePositiveGround + falseNegativeGround) else 0.0
        ioug = truePositiveGround / (falsePositiveGround + gtgroundsumadd) if (falsePositiveGround + gtgroundsumadd) else 0.0

        self.get_logger().info(f"Precision\t\t{precision:2.2%}\t\t{truePositiveGround}\t{falsePositiveGround}")
        self.get_logger().info(f"Recall   \t\t{recall:2.2%}\t\t{truePositiveGround}\t{falseNegativeGround}")
        self.get_logger().info(f"F1       \t\t{f1:2.2%}\t\t{falsePositiveGround}\t{falseNegativeGround}")
        self.get_logger().info(f"Accuracy\t\t{accuracy:2.2%}\t\t{(truePositiveGround + trueNegativeGround)}\t{(truePositiveGround + trueNegativeGround + falsePositiveGround + falseNegativeGround)}")
        self.get_logger().info(f"IoUg     \t\t{ioug:2.2%}")

def main():
    rclpy.init()
    node = EvalGroundpointClassifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_statistics()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
