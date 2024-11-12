import numpy as np
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
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
    transform.header.frame_id = self.odom_frame
    transform.child_frame_id = self.local_frame

    transform.transform.translation.x = float(pose[0])
    transform.transform.translation.y = float(pose[1])

    q = quaternion_from_euler(0, 0, pose[2])
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]

    # Broadcast transform
    self.tf_broadcaster.sendTransform(transform)

def publish_tags(self):
    marker_array_tags = MarkerArray()
    for i, tag in enumerate(self.tags_poses):
        marker_tag = Marker()
        marker_tag.header.frame_id = self.odom_frame
        marker_tag.header.stamp = self.get_clock().now().to_msg()
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
    self.tags_pub.publish(marker_array_tags)