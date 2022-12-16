import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rrbot_services.srv import EEJointVel, JointEEVel
from numpy import cos, sin, array, transpose, linalg

class VelConv(Node):

    def __init__(self):
        super().__init__('vel_converter')

        self.j_pos = array([0.0, 0.0, 0.0])
        self.j_vel = array([0.0, 0.0, 0.0])
        self.j_len = array([1.0, 1.0, 1.0])

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joints_callback,
            10)

        self.srv1 = self.create_service(EEJointVel, 'ee_joint_vel', self.calc_joint_vel)
        self.srv2 = self.create_service(JointEEVel, 'joint_ee_vel', self.calc_ee_vel)

    def joints_callback(self, msg):

        self.j_pos = msg.position
        self.j_vel = msg.velocity
    
    def calc_joint_vel(self, request, response):

        self.jacobian = array([

                    [- self.j_len[0]*sin(self.j_pos[0]) - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]), - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]),  0],

                    [  self.j_len[0]*cos(self.j_pos[0]) + self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),   self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),  0],

                    [                                                             0,                                               0, -1],

                    [                                                             0,                                               0,  0],

                    [                                                             0,                                               0,  0],

                    [                                                             1,                                               1,  0]
                ])

        joint_vels = linalg.pinv(self.jacobian) @ transpose(array([request.x_lvel, request.y_lvel, request.z_lvel, request.x_avel, request.y_avel, request.z_avel]))

        response.q1_vel = joint_vels[0]
        response.q2_vel = joint_vels[1]
        response.q3_vel = joint_vels[2]

        return response

    def calc_ee_vel(self, _, response):

        self.jacobian = array([

                    [- self.j_len[0]*sin(self.j_pos[0]) - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]), - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]),  0],

                    [  self.j_len[0]*cos(self.j_pos[0]) + self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),   self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),  0],

                    [                                                             0,                                               0, -1],

                    [                                                             0,                                               0,  0],

                    [                                                             0,                                               0,  0],

                    [                                                             1,                                               1,  0]
                ])

        ee_vel = self.jacobian @ transpose(self.j_vel)

        response.x_lvel = ee_vel[0]
        response.y_lvel = ee_vel[1]
        response.z_lvel = ee_vel[2]
        response.x_avel = ee_vel[3]
        response.y_avel = ee_vel[4]
        response.z_avel = ee_vel[5]

        return response

def main(args=None):
    rclpy.init(args=args)

    # Initialize Node
    vel_conv = VelConv()

    # Keep node running asynchronously
    rclpy.spin(vel_conv)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_conv.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()