import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from numpy import array, cos, sin, linalg, transpose
import sys

class Pd_vel(Node):

    def __init__(self):
        super().__init__('pd_vel_controller')

        self.j_effort = Float64MultiArray()
        self.j_effort.data = [0.0,0.0,0.0]
        self.j_pos = [0.0,0.0,0.0]
        self.j_vel = [0.0,0.0,0.0]
        self.j_len = [1, 1, 1]

        self.error = [0, 0, 0]
        self.qdd = [0.0, 0.0, 0.0]
        self.ee_vel = []

        for index, a in enumerate(sys.argv[1:]):
            self.ee_vel.append(float(a))
        # self.ee_vel = int(sys.argv[1:])

        self.kp = [1, 5, 3]
        self.kd = [8, 7, 8]

        # Inititialize FK listener
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joints_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_effort_controller/commands', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        if sum(self.error) != 0:

            self.jacobian = array([

                    [- self.j_len[0]*sin(self.j_pos[0]) - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]), - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]),  0],

                    [  self.j_len[0]*cos(self.j_pos[0]) + self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),   self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),  0],

                    [                                                             0,                                               0, -1],

                    # [                                                             0,                                               0,  0],

                    # [                                                             0,                                               0,  0],

                    # [                                                             1,                                               1,  0]
                ])

            self.qdd = linalg.pinv(self.jacobian) @ transpose(array(self.ee_vel))

            for i in range(len(self.kp)):

                self.error[i] = self.j_vel[i] - self.qdd[i]
                # print(self.ee_vel)

            self.j_effort.data[0] = (self.error[0] * self.kp[0]) #+ self.j_vel[i] * self.kd[i])
            self.j_effort.data[1] = -(self.error[1] * self.kp[1])
            self.j_effort.data[2] = (self.error[2] * self.kp[2])

        else:
            self.j_effort.data = [0.0,0.0,0.0]

            self.jacobian = array([

                    [- self.j_len[0]*sin(self.j_pos[0]) - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]), - self.j_len[1]*cos(self.j_pos[0])*sin(self.j_pos[1]) - self.j_len[1]*cos(self.j_pos[1])*sin(self.j_pos[0]),  0],

                    [  self.j_len[0]*cos(self.j_pos[0]) + self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),   self.j_len[1]*cos(self.j_pos[0])*cos(self.j_pos[1]) - self.j_len[1]*sin(self.j_pos[0])*sin(self.j_pos[1]),  0],

                    [                                                             0,                                               0, -1],

                    # [                                                             0,                                               0,  0],

                    # [                                                             0,                                               0,  0],
    
                    # [                                                             1,                                               1,  0]
                ])

            self.qdd = linalg.pinv(self.jacobian) @ transpose(array(self.ee_vel))

            for i in range(len(self.kp)):

                self.error[i] = self.j_vel[i] - self.qdd[i]
                # print(self.error)

        self.publisher_.publish(self.j_effort)
        print(self.jacobian)


    def joints_callback(self, msg):

        self.j_pos = msg.position
        self.j_vel = msg.velocity
        # self.curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec/10**9
   
def main(args=None):
    rclpy.init(args=args)

    # Initialize Node
    pd_vel = Pd_vel()

    # Keep node running asynchronously
    rclpy.spin(pd_vel)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pd_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()