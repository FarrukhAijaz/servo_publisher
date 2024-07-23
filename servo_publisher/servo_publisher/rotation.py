from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import math
from time import sleep
import rclpy
from std_msgs.msg import Float64

MIN_PULSE_WIDTH = 500 / 1_000_000  # For MG996R
MAX_PULSE_WIDTH = 2500 / 1_000_000  # For MG996R
SERVO_PIN = 12
ROTATION_SPEED = 0.1  # Adjust the rotation speed as needed
ANGLE = 10
OFFSET = 20 # Usually zero but I accidentaly glued it without referencing 
ROT_ANGLE_MIN= ANGLE - OFFSET
ROT_ANGLE_MAX= ANGLE + OFFSET

def main():
    rclpy.init()
    node = rclpy.create_node('servo_publisher')

    factory = PiGPIOFactory()
    servo = Servo(SERVO_PIN, min_pulse_width=MIN_PULSE_WIDTH, max_pulse_width=MAX_PULSE_WIDTH, pin_factory=factory)
    
    publisher = node.create_publisher(Float64, 'servo_angle', 10)

    try:
        while rclpy.ok():
            # Rotate from -10 to 10 degrees
            for ver_angle in range(-ROT_ANGLE_MIN , ROT_ANGLE_MAX + 1):  
                servo.value = math.sin(math.radians(ver_angle))
                sleep(ROTATION_SPEED)
                # Publish current servo angle
                angle_msg = Float64()
                angle_msg.data = float(ver_angle-OFFSET)
                publisher.publish(angle_msg)
                node.get_logger().info(f'Servo angle: {ver_angle-OFFSET} degrees')
            
            # Rotate back from 10 to -10 degrees
            for ver_angle in range(ROT_ANGLE_MAX, -(ROT_ANGLE_MIN-1), -1):
                servo.value = math.sin(math.radians(ver_angle))
                sleep(ROTATION_SPEED)
                # Publish current servo angle
                angle_msg = Float64()
                angle_msg.data = float(ver_angle-OFFSET)
                publisher.publish(angle_msg)
                node.get_logger().info(f'Servo angle: {ver_angle-OFFSET} degrees')

    except KeyboardInterrupt:
        pass

    finally:
        servo.detach()  # Stop the servo
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

