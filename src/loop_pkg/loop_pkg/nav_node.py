import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import random


class LoopNavNode(Node):
    def __init__(self):
        super().__init__('loop_nav_node')
#The lines above are what I learned to be the standard way to create a node
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10) #This line tells gazebo the velocity of the rover
        self.odom_sub = self.create_subscription(Odometry, '/model/ELM4_Chassis/odometry', self.odom_callback, 10)
        self.create_timer(0.1, self.control_loop)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.is_avoiding = False


        self.obstacles = [
            (5.0, 2.0),
            (-2.0, 3.0),
            (4.0, -4.0),
            (7.0, 6.0),
            (-5.0, -2.0),
            (1.0, 8.0),
            (-6.0, 5.0)
        ] #all the obstacles on the course hardcoded
    
        self.waypoints = [] #left empty so it generates later
        self.current_target_index = 0

        while len(self.waypoints) < 3:
            rx = random.uniform(-10.0, 10.0) #my world is 20 by 20, so 10 on each side of the origin is my range
            ry = random.uniform(-10.0, 10.0)

            safe = True
            for ox, oy in self.obstacles:
                if math.hypot(rx - ox, ry - oy) <= 2.0: #This for loop is used to identify if the rover is safe or not depending on how close it is to an obstacle
                    safe = False
                    break

            if safe:
                self.waypoints.append((rx,ry)) #now the coordinates for the waypoints are set!

#this isn't so necessary but I really like having it for display, the next line is gonna print in the terminal where its going.
        self.get_logger().info(f"Target Waypoints: {self.waypoints}")


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y) #position on the Y coordinate
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z) #position on the X coordinate
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp) #direction the rover is facing

    def get_distance(self, target_x, target_y):
        return math.hypot(target_y - self.current_y, target_x - self.current_x)
    

    def control_loop(self):
        msg = Twist()
        if self.current_target_index >= len(self.waypoints):
            self.get_logger().info("Mission Complete. Stopping.")
            self.cmd_pub.publish(msg)
            return

        target_x, target_y = self.waypoints[self.current_target_index]

        current_threshold = 1.4 if self.is_avoiding else 1.05

        obstacle_in_way = False
        danger_ox = 0.0
        danger_oy = 0.0

        for ox, oy in self.obstacles:
            if self.get_distance(ox, oy) <= current_threshold:
                obstacle_in_way = True
                danger_ox = ox
                danger_oy = oy
                break

        if obstacle_in_way:
            if not self.is_avoiding:
                self.get_logger().info(f"DANGER DETECTED! taking the fastest alternative route!")
                self.is_avoiding = True

            obs_yaw = math.atan2(danger_oy - self.current_y, danger_ox - self.current_x) #figuring out where the obstacle is
            target_yaw = math.atan2(target_y - self.current_y, target_x - self.current_x)
            escape_angle = target_yaw - obs_yaw
            escape_angle = math.atan2(math.sin(escape_angle), math.cos(escape_angle))
        
            if escape_angle > 0:
                avoidance_yaw = obs_yaw + 1.57
            else:
                avoidance_yaw = obs_yaw - 1.57
            
            heading_error = avoidance_yaw - self.current_yaw
            heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
        
            msg.angular.z = 1.0 * heading_error
            msg.linear.x = 0.3 

            self.cmd_pub.publish(msg)
            return

        if self.get_distance(target_x, target_y) < 0.4:
            self.get_logger().info(f"Reached waypoint {self.current_target_index + 1}")
            self.current_target_index += 1
            return

        if self.is_avoiding:
            self.get_logger().info("Obstacle cleared. Back on track!")
            self.is_avoiding = False

        target_yaw = math.atan2(target_y - self.current_y, target_x - self.current_x)
        heading_error = target_yaw - self.current_yaw
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        msg.angular.z = 0.5 * heading_error
        msg.linear.x = 0.5

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = LoopNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()