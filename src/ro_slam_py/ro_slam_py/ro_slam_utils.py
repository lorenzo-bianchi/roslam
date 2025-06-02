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

def publish_tags(self):
    marker_array_tags = MarkerArray()
    stamp = self.get_clock().now().to_msg()
    for i, tag in enumerate(self.tags_poses):
        marker_tag = Marker()
        marker_tag.header.frame_id = self.frame_odom
        marker_tag.header.stamp = stamp
        marker_tag.ns = 'tag'
        marker_tag.id = i
        marker_tag.type = Marker.SPHERE
        marker_tag.action = Marker.ADD
        marker_tag.pose.position.x = float(tag['x'])
        marker_tag.pose.position.y = float(tag['y'])
        if tag['last_hyp']:
            marker_size = 0.2
        else:
            marker_size = 0.1
        if self.show_tags_var:
            marker_tag.scale.x = marker_size * np.maximum(1.0, tag['var_x'])
            marker_tag.scale.y = marker_size * np.maximum(1.0, tag['var_y'])
        else:
            marker_tag.scale.x = marker_size
            marker_tag.scale.y = marker_size
        marker_tag.scale.z = marker_size
        marker_tag.color.a = 1.0
        marker_tag.color.r = self.robot_color[0]
        marker_tag.color.g = self.robot_color[1]
        marker_tag.color.b = self.robot_color[2]
        marker_array_tags.markers.append(marker_tag)

    if self.robot_id == 1:
        landmarks = [
            [0.00, 0.00],
            [1.79, 0.00],
            [1.72, 3.69],
            [-0.15, 3.72],
            [2.10, 1.86],
            [0.75, 3.12],
            [-0.34, 2.20],
            [0.88, 1.24],
        ]

        for i, landmarks_pos in enumerate(landmarks):
            marker_gt_landmark = Marker()
            marker_gt_landmark.header.frame_id = 'map'
            marker_gt_landmark.header.stamp = stamp
            marker_gt_landmark.ns = 'gt_landmarks'
            marker_gt_landmark.id = i
            marker_gt_landmark.type = Marker.SPHERE
            marker_gt_landmark.action = Marker.ADD
            marker_gt_landmark.pose.position.x = float(landmarks_pos[0])
            marker_gt_landmark.pose.position.y = float(landmarks_pos[1])
            marker_gt_landmark.pose.position.z = 0.3
            marker_gt_landmark.scale.x = 0.1
            marker_gt_landmark.scale.y = 0.1
            marker_gt_landmark.scale.z = 0.1
            marker_gt_landmark.color.a = 1.0
            marker_gt_landmark.color.r = 0.0
            marker_gt_landmark.color.g = 0.0
            marker_gt_landmark.color.b = 0.0
            marker_array_tags.markers.append(marker_gt_landmark)

    self.tags_pub.publish(marker_array_tags)
