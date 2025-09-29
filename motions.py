# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...


CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher=self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        qos=QoSProfile(reliability=2, durability=2, history=1, depth=10) # from slide 23

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        self.create_subscription(Imu, "/imu", self.imu_callback, qos) # usually only set qos for subscriber not publisher
        
        # ENCODER subscription
        self.create_subscription(Odometry, "/odom", self.odom_callback, qos)
        ...
        
        # LaserScan subscription 
        self.create_subscription(LaserScan, "/scan", self.laser_callback, qos)
        ...
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        self.imu_initialized = True
        # log imu msgs
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds

        # Get message data
        imu_angular_velocity = imu_msg.angular_velocity
        imu_linear_acceleration_x = imu_msg.linear_acceleration.x
        imu_linear_acceleration_y = imu_msg.linear_acceleration.y

        self.imu_logger.log_values([imu_linear_acceleration_x, imu_linear_acceleration_y, imu_angular_velocity, timestamp])
        

    def odom_callback(self, odom_msg: Odometry):
        self.odom_initialized = True
        # log odom msgs
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        # Get message data
        odom_orientation = odom_msg.pose.pose.orientation
        odom_x_pos = odom_msg.pose.pose.position.x
        odom_y_pos = odom_msg.pose.pose.position.y

        quaternion = [
            odom_orientation.x,
            odom_orientation.y,
            odom_orientation.z,
            odom_orientation.w
        ]
        th = euler_from_quaternion(quaternion)

        self.imu_logger.log_values([odom_x_pos, odom_y_pos, th, timestamp])
                
    def laser_callback(self, laser_msg: LaserScan):
        self.laser_initialized = True
        # log laser msgs with position msg at that time
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds

        # Get message data
        laser_range = laser_msg.ranges
        laser_angle_increment = laser_msg.angle_increment

        self.laser_logger.log_values([list(laser_range), laser_angle_increment, timestamp])


    def timer_callback(self):
        # Publishes messages here
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def make_circular_twist(self):
        msg=Twist()
        # fill up the twist msg for circular motion
        msg.linear.x = 0.5
        msg.angular.z = 0.7
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        # fill up the twist msg for spiral motion
        msg.angular.z = 0.7
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        # fill up the twist msg for line motion
        msg.linear.x = 0.5
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {args.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
