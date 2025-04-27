import rclpy
from rclpy.node import Node
from smbus2 import SMBus
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion
from sensor_msgs.msg import Imu
import math
import time
import numpy as np

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_imu_node')
        
        # Initialize I2C bus
        self.bus = SMBus(1)  # Use 1 for Raspberry Pi, 0 for some other boards
        self.mpu_addr = 0x68  # MPU6050 I2C address
        
        # Initialize MPU6050
        self.init_mpu()
        
        # Initialize transform broadcaster and IMU publisher
        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data', 10)
        
        # Create timer for publishing IMU data
        self.timer = self.create_timer(0.02, self.publish_imu_data)  # 50Hz update rate
        
        # Variables for complementary filter
        self.last_time = time.time()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        # Fixed covariance matrices (use moderate values instead of very small ones)
        # -1 means "covariance unknown" in ROS
        self.orientation_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.angular_velocity_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        self.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]
        
        self.get_logger().info('MPU6050 IMU node initialized')
        
    def init_mpu(self):
        # Wake up MPU6050
        self.bus.write_byte_data(self.mpu_addr, 0x6B, 0)
        
        # Configure accelerometer (+/- 2g)
        self.bus.write_byte_data(self.mpu_addr, 0x1C, 0)
        
        # Configure gyroscope (+/- 250 deg/s)
        self.bus.write_byte_data(self.mpu_addr, 0x1B, 0)
    
    def read_raw_data(self, addr):
        # Read high and low bytes
        high = self.bus.read_byte_data(self.mpu_addr, addr)
        low = self.bus.read_byte_data(self.mpu_addr, addr + 1)
        
        # Combine high and low bytes
        value = (high << 8) | low
        
        # Convert to signed value
        if value > 32767:
            value -= 65536
        return value
    
    def get_sensor_data(self):
        # Read accelerometer data
        acc_x = self.read_raw_data(0x3B) / 16384.0  # Scale for +/- 2g
        acc_y = self.read_raw_data(0x3D) / 16384.0
        acc_z = self.read_raw_data(0x3F) / 16384.0
        
        # Read gyroscope data
        gyro_x = self.read_raw_data(0x43) / 131.0  # Scale for +/- 250 deg/s
        gyro_y = self.read_raw_data(0x45) / 131.0
        gyro_z = self.read_raw_data(0x47) / 131.0
        
        # Convert gyro values to radians/sec
        gyro_x_rad = math.radians(gyro_x)
        gyro_y_rad = math.radians(gyro_y)
        gyro_z_rad = math.radians(gyro_z)
        
        return acc_x, acc_y, acc_z, gyro_x_rad, gyro_y_rad, gyro_z_rad
    
    def complementary_filter(self, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z):
        # Calculate time delta
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Calculate roll and pitch from accelerometer
        acc_roll = math.atan2(acc_y, acc_z)
        acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z))
        
        # Complementary filter
        alpha = 0.96  # Filter coefficient
        self.roll = alpha * (self.roll + gyro_x * dt) + (1 - alpha) * acc_roll
        self.pitch = alpha * (self.pitch + gyro_y * dt) + (1 - alpha) * acc_pitch
        self.yaw += gyro_z * dt
    
    def publish_imu_data(self):
        try:
            # Get sensor data
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.get_sensor_data()
            
            # Update orientation using complementary filter
            self.complementary_filter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
            
            # Create and fill IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Create quaternion from roll, pitch, yaw
            cr = math.cos(self.roll * 0.5)
            sr = math.sin(self.roll * 0.5)
            cp = math.cos(self.pitch * 0.5)
            sp = math.sin(self.pitch * 0.5)
            cy = math.cos(self.yaw * 0.5)
            sy = math.sin(self.yaw * 0.5)
            
            # Set orientation
            imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
            imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
            imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
            imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
            
            # Set covariances - use list instead of numpy array
            for i in range(9):
                imu_msg.orientation_covariance[i] = float(self.orientation_covariance[i])
                imu_msg.angular_velocity_covariance[i] = float(self.angular_velocity_covariance[i])
                imu_msg.linear_acceleration_covariance[i] = float(self.linear_acceleration_covariance[i])
            
            # Set angular velocity
            imu_msg.angular_velocity.x = float(gyro_x)
            imu_msg.angular_velocity.y = float(gyro_y)
            imu_msg.angular_velocity.z = float(gyro_z)
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = float(acc_x * 9.81)  # Convert to m/s^2
            imu_msg.linear_acceleration.y = float(acc_y * 9.81)
            imu_msg.linear_acceleration.z = float(acc_z * 9.81)
            
            # Publish IMU message
            self.imu_publisher.publish(imu_msg)
            
            # Also broadcast transform for visualization
            t = TransformStamped()
            t.header.stamp = imu_msg.header.stamp
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'imu_link'
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            t.transform.rotation = imu_msg.orientation
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing IMU data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()