import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import numpy as np
import matplotlib.pyplot as plt
import math
import numpy as np
import quaternion  # This library must be installed via `pip install numpy-quaternion`


class TF2AnglePlotter(Node):
    def __init__(self):
        super().__init__('tf2_angle_plotter')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust the timer interval as needed
        self.fig, self.ax = plt.subplots()
        self.times = []
        self.angles = []
        self.start_time = self.get_clock().now().nanoseconds
        self.tracker = AngleTracker()

    def timer_callback(self):
        try:
            # Change 'arm_tip' and 'world' to your tf names as needed
            transform: TransformStamped = self.tf_buffer.lookup_transform('rail', 'arm_tip', rclpy.time.Time())
            current_time = self.get_clock().now().nanoseconds
            time_sec = (current_time - self.start_time) * 1e-9  # Convert nanoseconds to seconds
            
            # Extract the z-axis component from the quaternion
            quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]

            world_z_axis = np.array([0, 0, 1])

            arm_z_axis = self.rotate_vector_by_quaternion(world_z_axis, quaternion)

            # Calculate the angle
            #angle = np.arccos(np.dot(arm_z_axis, world_z_axis) / (np.linalg.norm(arm_z_axis) * np.linalg.norm(world_z_axis)))

            angle = self.angle_between_vectors_clockwise(arm_z_axis, world_z_axis)

            
            total_rotation = self.tracker.update_angle(np.degrees(angle))

            self.angles.append(total_rotation)
            self.times.append(time_sec)

            # Plotting
            self.ax.clear()
            self.ax.plot(self.times, self.angles)
            self.ax.set_xlabel('Time (s)')
            self.ax.set_ylabel('Angle (degrees)')
            plt.pause(0.001)

        except tf2_ros.LookupException:
            self.get_logger().error('Transform not found')
    
    def rotate_vector_by_quaternion(self, vector, quaternion):
        # Normalize the quaternion to ensure it's a unit quaternion
        q = np.quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        q_normalized = q.normalized()

        # Convert the vector into a quaternion with zero real part
        v = np.quaternion(0, vector[0], vector[1], vector[2])

        # Perform the quaternion multiplication (rotation)
        v_rot = q_normalized * v * q_normalized.inverse()

        # Return the rotated vector components
        return np.array([v_rot.x, v_rot.y, v_rot.z])

    def angle_between_vectors_clockwise(self, v1, v2, normal_direction=np.array([0, -1, 0])):
        # Calculate the dot product
        dot_product = np.dot(v1, v2)

        # Calculate the norms of each vector
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        # Calculate the cosine of the angle
        cos_theta = dot_product / (norm_v1 * norm_v2)
        cos_theta = max(min(cos_theta, 1), -1)  # Clamp value to the range [-1, 1] for numerical stability

        # Calculate the angle in radians
        angle_rad = np.arccos(cos_theta)

        # Determine the direction using the cross product and the reference normal_direction vector
        cross_product = np.cross(v1, v2)
        direction = np.dot(cross_product, normal_direction)

        # If the direction is negative, adjust the angle to reflect a clockwise sweep past 180 degrees
        if direction < 0:
            angle_rad = 2 * np.pi - angle_rad

        # Convert radians to degrees
        angle_deg = np.degrees(angle_rad)

        return angle_rad

def main(args=None):
    rclpy.init(args=args)
    node = TF2AnglePlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


class AngleTracker:
    def __init__(self):
        self.previous_angle = 0
        self.total_rotation = 0

    def update_angle(self, new_angle):
        delta_angle = new_angle - self.previous_angle
        
        # Check for wrapping
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360

        self.total_rotation += delta_angle
        self.previous_angle = new_angle

        # Return the total rotation to keep track of how many times and in which direction zero has been crossed
        return self.total_rotation
