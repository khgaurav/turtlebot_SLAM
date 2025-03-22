#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class DepthProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.target_size = (84, 84)
        self.latest_depth = None
        
    def depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depth_array = np.nan_to_num(cv_image.astype(np.float32))
        resized = cv2.resize(depth_array, self.target_size, interpolation=cv2.INTER_NEAREST)
        self.latest_depth = resized.squeeze()
        
        # cv2.imshow('Depth Preview', (resized/resized.max() * 255).astype(np.uint8))
        # cv2.waitKey(1)

class MotionController:
    def __init__(self):
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.cmd = Twist()
        self.speed = 0.2
        self.max_wheel_speed = 1
        self.wheel_separation = 0
        
    def set_wheel_speeds(self, left, right):
        v_left = left * self.max_wheel_speed
        v_right = right * self.max_wheel_speed
        linear_x = (v_right + v_left) / 2.0
        # angular_z = (v_right - v_left) / self.wheel_separation
        angular_z = 0
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        
        self.vel_pub.publish(twist)
        # rospy.loginfo(f"Command: linear={linear_x:.2f} m/s, angular={angular_z:.2f} rad/s")


class MainController:
    def __init__(self):
        rospy.init_node('turtlebot_navigator')
        
        # Initialize components
        self.depth_processor = DepthProcessor()
        self.motion_controller = MotionController()
        
    def run(self):
        while not rospy.is_shutdown():
            if self.depth_processor.latest_depth is not None:
                self.motion_controller.set_wheel_speeds(1, 1)
                rospy.loginfo("Maintaining forward motion")

if __name__ == '__main__':
    try:
        controller = MainController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
