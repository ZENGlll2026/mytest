from geometry_msgs.msg import PoseStamped,Pose
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
import rclpy.time
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PatrolNode(BasicNavigator):
    def __init__(self,node_name='patrol_node'):
        super().__init__(node_name)
        self.count_i=0
        self.declare_parameter('initial_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0, 3.0, 1.0, 1.57])
        self.declare_parameter('image_save_path','/home/sunrise/CHAPT7/chapt7_ws/src/autopatrol_robot/image')
        self.initial_point = self.get_parameter('initial_point').value
        self.target_points = self.get_parameter('target_points').value
        self.image_save_path = self.get_parameter('image_save_path').value
        self.get_logger().info(f'初始点: {self.initial_point}')
        self.get_logger().info(f'目标点: {self.target_points}')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.cv_bridge = CvBridge()
        self.latest_img_ = None
        self.img_sub_ = self.create_subscription(Image, '/camera_sensor/image_raw', self.image_callback, 1)
        
    def image_callback(self, msg):
        self.latest_img_ = msg
    def record_img(self):
        if self.latest_img_ is not None :
            pose = self.get_current_pose()
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_img_, desired_encoding='bgr8')
            cv2.imwrite(f'{self.image_save_path}/img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png', cv_image)
    def get_pose_by_xyyaw(self,x,y,yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.header.stamp = self.get_clock().now().to_msg()
        # 2. 欧拉角 → 四元数（核心！）
        # quaternion_from_euler(roll, pitch, yaw),返回参数顺序是xyzw,有些函数是wxyz,要注意区分
        quat = quaternion_from_euler(0, 0, yaw)
        # 3. 把四元数赋值给姿态
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose
    
    def init_robot_pose(self):
        
        self.initial_point_ = self.get_parameter('initial_point').value
        init_pose = self.get_pose_by_xyyaw(self.initial_point_[0], self.initial_point_[1], self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()
        
    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for i in range(0, len(self.target_points_), 3):
            x = self.target_points_[i]
            y = self.target_points_[i+1]
            yaw = self.target_points_[i+2]
            points.append([x, y, yaw])
        return points
    def nav_to_pose(self,target_point):
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if self.count_i >=10:
                self.count_i=0
                if feedback is not None:
                    self.get_logger().info(f'剩余距离: {feedback.distance_remaining:.2f} m')
            self.count_i+=1
        result = self.getResult()
        self.get_logger().info(f'导航结果: {result}')
        
    def get_current_pose(self):
        while rclpy.ok():
            try:
                result = self.buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = result.transform
                self.get_logger().info(f'平移:{transform.translation}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')
def main():
    rclpy.init()
    partol = PatrolNode() #节点

    partol.init_robot_pose()
    
    target_points = partol.get_target_points()
 
    for point in target_points:
        partol.get_logger().info(f'导航到目标点: {point}')
        target_pose = partol.get_pose_by_xyyaw(point[0], point[1], point[2])
        partol.nav_to_pose(target_pose)
        partol.get_logger().info(f'已经到达目标点{point[0]}_{point[1]},正在记录图像')
        partol.record_img()
        partol.get_logger().info(f'图像记录完成')
  
    rclpy.shutdown()
    