import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from ro_slam_interfaces.msg import Landmark, LandmarkArray
from transforms3d.euler import euler2quat
from visualization_msgs.msg import Marker, MarkerArray

def broadcast_pose(self):
    """
    Broadcast transform
    """
    # Get robot pose
    pose = self.fed_ekf.get_robot_pose()

    # Create transform
    transform = TransformStamped()

    transform.header.stamp = self.get_clock().now().to_msg()
    transform.header.frame_id = self.frame_odom
    transform.child_frame_id = self.frame_local

    transform.transform.translation.x = float(pose[0])
    transform.transform.translation.y = float(pose[1])

    q = euler2quat(0, 0, pose[2])
    transform.transform.rotation.w = q[0]
    transform.transform.rotation.x = q[1]
    transform.transform.rotation.y = q[2]
    transform.transform.rotation.z = q[3]

    # Broadcast transform
    self.tf_broadcaster.sendTransform(transform)

def publish_pose_landmarks(self):
    """
    Publish pose and landmarks
    """
    # Publish pose
    pose = self.fed_ekf.get_robot_pose()

    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.stamp = self.get_clock().now().to_msg()
    pose_msg.header.frame_id = self.frame_odom

    pose_msg.pose.pose.position.x = float(pose[0])
    pose_msg.pose.pose.position.y = float(pose[1])

    q = euler2quat(0, 0, pose[2])
    pose_msg.pose.pose.orientation.w = q[0]
    pose_msg.pose.pose.orientation.x = q[1]
    pose_msg.pose.pose.orientation.y = q[2]
    pose_msg.pose.pose.orientation.z = q[3]

    pose_msg.pose.covariance[0] = self.fed_ekf.P[0, 0]
    pose_msg.pose.covariance[7] = self.fed_ekf.P[1, 1]
    pose_msg.pose.covariance[35] = self.fed_ekf.P[2, 2]

    self.pose_pub.publish(pose_msg)

    # publish landmarks
    landmark_array_msg = LandmarkArray()
    landmark_array_msg.header.stamp = self.get_clock().now().to_msg()
    landmark_array_msg.id = self.robot_id

    tags_poses = self.fed_ekf.get_tags_poses()
    for tag in tags_poses:
        landmark = Landmark()
        landmark.x = float(tag['x'])
        landmark.y = float(tag['y'])
        landmark.var_x = float(tag['var_x'])
        landmark.var_y = float(tag['var_y'])
        landmark.cov_xy = float(tag['cov_xy'])
        landmark_array_msg.landmarks.append(landmark)

    self.landmark_est_pub.publish(landmark_array_msg)
