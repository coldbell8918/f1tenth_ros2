# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy, math
from rclpy.node import Node

from geometry_msgs.msg import Twist

from ackermann_msgs.msg import AckermannDriveStamped
import rclpy.time

class CmdtoAckermann(Node):

  def __init__(self):

    super().__init__('ack_publisher')

    self.wheelbase = 0.3302
    
    self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10)
    
    self.subscription

    self.frame_id = 'odom'

    self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

  def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
    if omega == 0 or v == 0:
      return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


  def cmd_callback(self, data):
   
    v = data.linear.x
    steering = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z, self.wheelbase)
    
    self.msg = AckermannDriveStamped()
    
    self.msg.header.frame_id = self.frame_id
    self.msg.drive.steering_angle = steering
    self.msg.drive.speed = v
    
    self.publisher_.publish(self.msg)

def main(args=None):

  rclpy.init(args=args)
  cmd_to_acker = CmdtoAckermann()
  rclpy.spin(cmd_to_acker)

  cmd_to_acker.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__': 
  main()