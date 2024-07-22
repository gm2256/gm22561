# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import lgpio
import time

GPIO_TRIGGER = 18
GPIO_ECHO = 24

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # init GPIO pins
        self.h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self.h, GPIO_TRIGGER)
        lgpio.gpio_claim_input(self.h, GPIO_ECHO)


        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        timer_period = 1/10  # data rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        lgpio.gpio_write(self.h, GPIO_TRIGGER, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, GPIO_TRIGGER, 0)

        startTime = time.time()
        endTime = time.time()

        while lgpio.gpio_read(self.h, GPIO_ECHO) == 0:
            startTime = time.time()

        while lgpio.gpio_read(self.h, GPIO_ECHO) == 1:
            endTime = time.time()

        timeDiff = endTime - startTime
        distance_cm = timeDiff * 34300 / 2

        msg = Float32()
        msg.data = distance_cm
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    lgpio.gpiochip_close(minimal_publisher.h)
    rclpy.shutdown()


if __name__ == '__main__':
    main()