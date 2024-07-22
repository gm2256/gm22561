import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import RPi.GPIO as GPIO

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        self.publisher_ = self.create_publisher(String, 'motor_status', 10)
        self.subscription = self.create_subscription(String, 'motor_command', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # GPIO 설정
        self.motor_pins = {'IN1': 22, 'IN2': 23, 'ENA': 24}
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.motor_pins['IN1'], GPIO.OUT)
        GPIO.setup(self.motor_pins['IN2'], GPIO.OUT)
        GPIO.setup(self.motor_pins['ENA'], GPIO.OUT)
        GPIO.output(self.motor_pins['ENA'], GPIO.HIGH)  # ENA 핀을 상시 HIGH로 설정

    def listener_callback(self, msg):
        command = msg.data
        if command == 'forward':
            GPIO.output(self.motor_pins['IN1'], GPIO.HIGH)
            GPIO.output(self.motor_pins['IN2'], GPIO.LOW)
        elif command == 'backward':
            GPIO.output(self.motor_pins['IN1'], GPIO.LOW)
            GPIO.output(self.motor_pins['IN2'], GPIO.HIGH)
        elif command == 'stop':
            GPIO.output(self.motor_pins['IN1'], GPIO.LOW)
            GPIO.output(self.motor_pins['IN2'], GPIO.LOW)

        self.publisher_.publish(String(data='Command executed: ' + command))

    def destroy_node(self):
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
    rclpy.spin(motor_control_node)
    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





