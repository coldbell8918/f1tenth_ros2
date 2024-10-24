import numpy as np
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class ReactiveFollowGap(Node):

    BUBBLE_RADIUS = 2
    PREPROCESS_CONV_SIZE = 5
    BEST_POINT_CONV_SIZE = 80
    STRAIGHTS_STEERING_ANGLE = np.pi / 12  # 15 degrees
    MAX_LIDAR_DIST = 20
    WEIGT_ANGLE = 2.5 # 클 수록 적게 회전
    RATIO = 1.2

    def __init__(self):

        super().__init__('reactive_node')

        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
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

    def cluster_consecutive(self,indices, min_length=60):
        clusters = np.split(indices, np.where(np.diff(indices) != 1)[0] + 1)
        return [cluster for cluster in clusters if len(cluster) >= min_length]
    
    def find_bounds_of_largest_cluster(self, clusters):
    
        largest_indices = [(max(cluster), cluster) for cluster in clusters]
        
        valid_clusters = [cluster for largest_index, cluster in largest_indices if largest_index <= 800]
        
        if valid_clusters:
            next_largest_cluster = max(valid_clusters, key=lambda cluster: max(cluster))
            
            smallest_index = min(next_largest_cluster)
            largest_index = max(next_largest_cluster)
            
            return smallest_index, largest_index

    def scan_callback(self, scan_msg):
        
        self.radians_per_elem = (1.5 * np.pi) / len(scan_msg.ranges)
        proc_ranges = np.array(scan_msg.ranges[135:-135])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)

        filtered_indices = np.where(proc_ranges >= 2.0)[0]
        clusters = self.cluster_consecutive(filtered_indices)

        left = scan_msg.ranges[720]
        right = scan_msg.ranges[380]
        # step = scan_msg.ranges[540]
        step = np.fabs(np.average(scan_msg.ranges[499:580]))
        
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

        if len(clusters)>=2:
            start, end = self.find_bounds_of_largest_cluster(clusters)
            best = self.find_best_point(start, end, proc_ranges)
        print('len: ', len(clusters))
        print('best point:', best)
        angle = self.get_angle(best, len(proc_ranges)) - (0.5 * (0.4 / left)) + (0.5 * (0.4 / right))

        # velocity = 0.0

        # if step >= 8.0:
        #     if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
        #         velocity = 5.0 * self.RATIO
        #     else:
        #         velocity = 9.0 * self.RATIO
        # elif 8.0 > step >= 5.0:
        #     if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
        #         velocity = 5.0 * self.RATIO
        #     else:
        #         velocity = 7.5 * (step / 8.0) * self.RATIO
        #         if velocity > (7.5 * self.RATIO):
        #             velocity = 7.5 * self.RATIO
        # elif 5.0 > step >= 2.0:
        #     if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
        #         velocity = 5.0 
        #     else:
        #         velocity = 6.0 * (step / 2.5) * self.RATIO
        #         if velocity > (6.0 * self.RATIO):
        #             velocity = 6.0 * self.RATIO
        # elif 2.0 > step >= 0.0:
        #     if abs(angle) > self.STRAIGHTS_STEERING_ANGLE:
        #         velocity = 3.0 
        #     else:
        #         velocity = 2.0 * (step / 1.0) 
        #         if velocity > 2.0:
        #             velocity = 2.0 
        # else:
        #     velocity = 4.0 * self.RATIO

        velocity = np.sqrt(2 * 0.523 * 9.81 * np.fabs(step))
        
        if velocity >= 11:
            velocity = 11

        

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
