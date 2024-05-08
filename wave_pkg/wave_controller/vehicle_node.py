import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import Imu
from wave_rover_serial import Robot
import math

class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')
        
        # Initialize connection to the vehicle
        self.rover = Robot('COM9')
        self.rover.connect()
        
        # Subscribe to the /cmd_vel topic to receive velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publish IMU data to the /imu_data topic
        self.imu_data_publisher = self.create_publisher(
            Imu,
            '/imu',
            10)
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Process velocity commands received from /cmd_vel topic
        """
        # Convert Twist message to JSON format compatible with wave_rover_serial
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        wheelbase = 0.136 # in meters
        left_speed = linear_velocity - angular_velocity * wheelbase/2
        right_speed = linear_velocity + angular_velocity * wheelbase / 2

        # Keeping the speed input in acceptable range
        left_speed = max(min(left_speed, 255), -255)
        right_speed = max(min(right_speed, 255), -255)

        self.rover.speed_input(left_speed, right_speed)
    
    def publish_imu_data(self):
        """
        Publishes the imu data to a the /imu_data topic
        """
        # Get IMU data from the vehicle
        imu_data = self.rover.imu_info()
        
        # Create an Imu message
        imu_msg = Imu()
        
        # Populate header (timestamp)
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Populate orientation (quaternion)
        quaternion = self.quaternion_from_rpy(math.radians(imu_data['roll']),
                                              math.radians(imu_data['pitch']),
                                              math.radians(imu_data['yaw']))
        imu_msg.orientation = Quaternion(*quaternion)
        
        # Populate orientation covariance
        imu_msg.orientation_covariance = [0.0] * 9  # Assuming no covariance for orientation
        
        # Populate angular velocity
        imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = imu_data['gyro_X']
        imu_msg.angular_velocity.y = imu_data['gyro_Y']
        imu_msg.angular_velocity.z = imu_data['gyro_Z']
        
        # Populate angular velocity covariance
        imu_msg.angular_velocity_covariance = [0.0] * 9  # Assuming no covariance for angular velocity
        
        # Populate linear acceleration
        imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = imu_data['acce_X']
        imu_msg.linear_acceleration.y = imu_data['acce_Y']
        imu_msg.linear_acceleration.z = imu_data['acce_Z']
        
        # Populate linear acceleration covariance
        imu_msg.linear_acceleration_covariance = [0.0] * 9  # Assuming no covariance for linear acceleration
        
        # Publish IMU message
        self.imu_data_publisher.publish(imu_msg)
    
    def quaternion_from_rpy(self, roll, pitch, yaw):
        """
        Convert roll, pitch, yaw to quaternion.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]
    
    def disconnect(self):
        # Disconnect from the vehicle when shutting down the node
        self.rover.disconnect()

def main(args=None):
    rclpy.init(args=args)
    vehicle_node = VehicleNode()
    
    try:
        rclpy.spin(vehicle_node)
    except KeyboardInterrupt:
        pass
    
    vehicle_node.disconnect()
    vehicle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()