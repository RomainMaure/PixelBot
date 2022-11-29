import rclpy
from rclpy.node import Node

from pixelbot_msgs.srv import MotorsMovement

from adafruit_servokit import ServoKit


class Motors(Node):

    # Defines
    RIGHT_ARM, LEFT_ARM, RIGHT_ANTENNA, LEFT_ANTENNA = 0, 1, 2, 3
    MOTOR_STRING_TO_DEFINE = {"right_arm": RIGHT_ARM,
                              "left_arm": LEFT_ARM,
                              "right_antenna": RIGHT_ANTENNA,
                              "left_antenna": LEFT_ANTENNA}

    def __init__(self):
        super().__init__('pixelbot_motors_node')

        # Service to perform the desired movement of given motors
        self.motors_movement_srv = self.create_service(MotorsMovement, 'motors_movement', self.motors_movement_callback)

        # Service to perform walking movement

        # Service to perform emotion with antennae movements

        # Servokit object
        self.kit = ServoKit(channels=16)

    
    def motors_movement_callback(self, request, response):
        """
        Service handler allowing to perform the desired movemement
        of given motors.

        :param request: See MotorsMovement message definition.
        :param response: See MotorsMovement message definition.
        """

        # Check request arguments
        if len(request.body_parts) != len(request.angles):
            self.get_logger().info("Error: The input lists should have the same length!")
            response.success = False
            return response

        for body_part, angle in zip(request.body_parts, request.angles):
            if body_part not in self.MOTOR_STRING_TO_DEFINE.keys():
                self.get_logger().info("Error: requested body part does not exist!")
                response.success = False
                return response

            if not 0 <= angle <= 180:
                self.get_logger().info("Error: requested angle should be in [0; 180]!")
                response.success = False
                return response

        # Perform the requested movements if the arguments are valid
        for body_part, angle in zip(request.body_parts, request.angles):
            self.kit.servo[self.MOTOR_STRING_TO_DEFINE[body_part]].angle = angle

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)

    motors_node = Motors()

    rclpy.spin(motors_node)

    motors_node.destroy_node() ########
    rclpy.shutdown()


if __name__ == '__main__':
    main()
