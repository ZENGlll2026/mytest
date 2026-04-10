from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    nav = BasicNavigator() #节点
    nav.waitUntilNav2Active()
    
    goal_poses = []
    goal_point_pose = [[1.0, 1.0, 1.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0]]
    for point in goal_point_pose:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.orientation.w = point[2]
    
        goal_poses.append(goal_pose)
    
    nav.followWaypoints(goal_poses)
    while not nav.isTaskComplete():
        Feedback = nav.getFeedback()
        nav.get_logger().info(f'终点编号: {Feedback.current_waypoint}')
    
    result = nav.getResult()
    nav.get_logger().info(f'导航结果: {result}')
    # rclpy.spin(nav)
    # rclpy.shutdown()
    