import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class RvizMarker(Node):
    def __init__(self):
        super().__init__('rviz_marker_node')
        self.marker_pub = self.create_publisher(Marker, 'frontier_chosen_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, 'frontier_all_marker', 10)

    def update_map_consts(self, map_res, map_origin_x, map_origin_y):
        self.map_res = map_res
        self.map_origin_x = map_origin_x
        self.map_origin_y = map_origin_y
        self.get_logger().info(f"Map consts updated: {self.map_res}, {self.map_origin_x}, {self.map_origin_y}")

    def create_marker(self, 
            position,
            frame_id="map", 
            marker_id=0, 
            marker_type=Marker.SPHERE,  
            orientation = (0.0, 0.0, 0.0, 1.0), 
            scale = (0.2, 0.2, 0.2), 
            color = (0.0, 0.6, 0.1, 0.1) #RGB-Alpha
            ):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontier"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(position[1] * self.map_res + self.map_origin_x)
        marker.pose.position.y = float(position[0] * self.map_res + self.map_origin_y)
        marker.pose.position.z = 0.0 #we don't have z axis in this project
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        return marker

    def publish_marker(self, chosen_frontier):
        marker_x = chosen_frontier[1] * self.map_res + self.map_origin_x
        marker_y = chosen_frontier[0] * self.map_res + self.map_origin_y
        self.get_logger().info(f"Publishing marker at frontier {chosen_frontier} coords: {marker_x}, {marker_y}")
        self.marker_pub.publish(self.create_marker(
            position=chosen_frontier, 
            color=(0.6, 0.0, 0.1, 1.0), 
            scale=(0.6, 0.6, 0.6)))

    def publish_marker_array(self, frontiers_list):
        self.get_logger().info(f"Publishing markers")
        marker_array = MarkerArray()
        for i, frontier in enumerate(frontiers_list):
            marker_array.markers.append(self.create_marker(marker_id=i, position=frontier))
        self.marker_array_pub.publish(marker_array)
