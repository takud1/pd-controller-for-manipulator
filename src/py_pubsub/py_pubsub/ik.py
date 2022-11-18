from rrbot_services.srv import Inverse
from numpy import arctan2, arccos, sqrt

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Inverse, 'conv_ik', self.calc_ik)

    def calc_ik(self, request, response):

        x = request.x1
        y = request.y1
        z = request.z1

        l1 = 1
        l2 = 1

        response.q3 = 1 - z
        D = (x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2)
        response.q2 = arccos(D)

        response.q1 = arctan2(y, x) - response.q2

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()