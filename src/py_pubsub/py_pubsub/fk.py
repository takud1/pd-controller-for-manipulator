import rclpy
from rclpy.node import Node
import numpy as np
from numpy import cos, sin, deg2rad, rad2deg, pi
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

class Frwrds(Node):

    def __init__(self):
        super().__init__('give_fk')

        # Inititialize FK listener
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.fk_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Pose, 'ee_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Link lengths
        self.l1 = 1
        self.l2 = 1
        self.l3 = 1

    def timer_callback(self):
        self.publisher_.publish(self.pose)

    # Forward Kinematics Callback Function
    def fk_callback(self, msg):
        self.calc_fk(msg.position)

    # Inverse Kinematics Callback Function
    def ik_callback(self, msg):
        print(self.calc_ik(msg.position.x, msg.position.y, msg.position.z))

    # Calculating FK
    def calc_fk(self, position):

        q1 = position[0]
        q2 = position[1] + pi
        q3 = position[2]

        # Homogeneous Transformation Matrix
        a1 = np.array([[cos(q1), -sin(q1), 0, self.l1*cos(q1)],
 		   	   [sin(q1), cos(q1), 0, self.l1*sin(q1)],
 		   	   [0, 0, 1, 0],
 		   	   [0, 0, 0, 1]])

        a2 = np.array([[cos(q2), sin(q2), 0, -self.l2*cos(q2)],
 		   	   [sin(q2), -cos(q2), 0, -self.l2*sin(q2)],
 		   	   [0, 0, -1, 0],
 		   	   [0, 0, 0, 1]])

        a3 = np.array([[1, 0, 0, 0],
 		   	   [0, 1, 0, 0],
 		   	   [0, 0, 1, q3],
 		   	   [0, 0, 0, 1]])
        
        t02 = a1 @ a2
        t03 = t02 @ a3

        conv = Rotation.from_matrix(t03[:3, :3])
        quat = conv.as_quat()
        d = t03[:-1, 3]

        self.pose = Pose()
        self.pose.position.x = d[0]
        self.pose.position.y = d[1]
        self.pose.position.z = d[2] + 1

        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]

        print(self.pose)

    # Calculating IK
    def calc_ik(self, x, y, z):

        # Angle 1 Calculation
        q1 = np.arctan2(y, x)

        # Angle 3 Calculation
        q3 = np.arccos(((x**2) + (y**2) + (z -self.l1) * (z -self.l1) - (self.l2**2) - (self.l3**2))/(2 * self.l2 * self.l3))

        # Angle 2 Calculations
        q2 = np.arctan2(z - self.l1,np.sqrt((x**2)+(y**2))) - np.arctan2(self.l3*sin(q3),(self.l3*cos(q3)) + self.l2)

        q2_neg = np.arctan2(z-self.l1,np.sqrt((x*x)+(y*y)))-np.arctan2(self.l3*sin(-1*q3),(self.l3*cos(-1*q3)) + self.l2 )
        
        return rad2deg(q1), rad2deg(q2), rad2deg(q3)

def main(args=None):
    rclpy.init(args=args)

    # Initialize Node
    frwrds = Frwrds()

    # Keep node running asynchronously
    rclpy.spin(frwrds)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frwrds.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()