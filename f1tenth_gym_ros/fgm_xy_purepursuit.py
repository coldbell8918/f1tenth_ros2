import numpy as np
from rclpy.node import Node
import math
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class ReactiveFollowGap(Node):

    BUBBLE_RADIUS = 2
    PREPROCESS_CONV_SIZE = 5
    BEST_POINT_CONV_SIZE = 80
    STRAIGHTS_STEERING_ANGLE = np.pi / 12  # 15 degrees
    MAX_LIDAR_DIST = 3000000
    LFD = 1.9

    # 지름길 입구 좌표
    X_GOAL = 3.840
    Y_GOAL = -4.696

    # 지름길 구간
    GLOBAL_MIN_X = 1.508
    GLOBAL_MAX_X = 4.303
    GLOBAL_MIN_Y = -6.556
    GLOBAL_MAX_Y = -4.819

    WEIGT_ANGLE = 2.0 # 클수록 적게 회전

    def __init__(self):
        
        super().__init__('reactive_node')
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 1)
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 1)
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

        self.ackermann_data = AckermannDriveStamped()
        self.ackermann_data.drive.acceleration = 0.0
        self.ackermann_data.drive.jerk = 0.0
        self.ackermann_data.drive.steering_angle = 0.0
        self.ackermann_data.drive.steering_angle_velocity = 0.0
        self.ackermann_data.drive.speed = 0.0

        self.current_odom_x = 0.0
        self.current_odom_y = 0.0

    def odom_callback(self, odom_msg):
        self.current_odom_x = odom_msg.pose.pose.position.x
        self.current_odom_y = odom_msg.pose.pose.position.y
      
    def scan_callback(self, scan_msg):
       
        self.radians_per_elem = (1.5 * np.pi) / len(scan_msg.ranges)
        proc_ranges = np.array(scan_msg.ranges[135:-135])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        

        left_ranges = scan_msg.ranges[680:761]
        left = sum(left_ranges) / len(left_ranges)

        
        right_ranges = scan_msg.ranges[340:421]
        right = sum(right_ranges) / len(right_ranges)
        
        step_ranges = scan_msg.ranges[500:581]
        step = sum(step_ranges) / len(step_ranges)
        
        closest = proc_ranges.argmin()
        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0:
            min_index = 0
        if max_index >= len(proc_ranges):
            max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)
        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        angle = self.get_angle(best, len(proc_ranges)) - (0.15 * (0.4 / left)) + (0.15 * (0.4 / right))

        
        a=math.atan((self.X_GOAL-self.current_odom_x)/(self.Y_GOAL-self.current_odom_y))
        true_angle=math.atan((2*0.25*math.sin(a))/self.LFD)
        
        if self.GLOBAL_MIN_X <= self.current_odom_x <= self.GLOBAL_MAX_X and self.GLOBAL_MIN_Y <= self.current_odom_y <= self.GLOBAL_MAX_Y:
          velocity=5.0
          angle=true_angle

        
        else :
              
          if step >= 8.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 5.0
              else:
                  velocity = 9.0
          elif 8.0 > step >= 5.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 5.0
              else:
                  velocity = 7.5 * (step / 8.0)
                  if velocity > 7.5:
                      velocity = 7.5
          elif 5.0 > step >= 2.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 5.0
              else:
                  velocity = 6.0 * (step / 5.0)
                  if velocity > 6.0:
                      velocity = 6.0
          elif 2.0 > step >= 0.0:
              if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
                  velocity = 2.0
              else:
                  velocity = 2.0 * (step / 2.0) 
                  if velocity > 2.0:
                      velocity = 2.0
          else:
              velocity = 4.0
         
        self.ackermann_data.drive.speed = velocity 
        self.ackermann_data.drive.steering_angle = angle
        self.ackermann_data.drive.steering_angle_velocity = 0.0
        self.ackermann_data.drive.acceleration = 0.0
        self.ackermann_data.drive.jerk = 0.0
        self.drive_publisher.publish(self.ackermann_data)

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):
        averaged_max_gap = np.convolve(
            ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE), 'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):
        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / self.WEIGT_ANGLE
        return steering_angle
    
def main(args=None):

    rclpy.init(args=args)
    print("FGM Initialized")
    fgm_node = ReactiveFollowGap()
    rclpy.spin(fgm_node)

    fgm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
