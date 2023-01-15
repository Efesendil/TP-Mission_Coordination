#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
from evry_project_plugins.srv import DistanceToFlag
import math


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0  # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0  # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

        # Set the attractive and repulsive gains
        self.k_att = 1.0
        self.k_rep = 1.0

        # Set the maximum distance at which the attractive potential
        # field has an effect
        self.r_att_max = 10.0

        # Set the maximum distance at which the repulsive potential
        # field has an effect
        self.r_rep_max = 5.0

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def compute_attractive_potential(self, goal):
        # Compute the distance to the goal
        dx = goal.x - self.x
        dy = goal.y - self.y
        distance = math.sqrt(dx**2 + dy**2)

    # Compute the attractive potential
        if distance > self.r_att_max:
            U_att = 0.0
        else:
            U_att = self.k_att * distance

        return U_att

    def compute_repulsive_potential(self, obstacle):
        # Initialize the repulsive potential to zero
        U_rep = 0.0
        # Compute the distance to the obstacle
        dx = obstacle.x - self.x
        dy = obstacle.y - self.y
        distance = math.sqrt(dx**2 + dy**2)
        # Compute the repulsive potential
        if distance > self.r_rep_max:
            U_rep = 0.0
        else:
            U_rep += self.k_rep * (1.0/distance - 1.0/self.r_rep_max)**2

        return U_rep

    def compute_potential_field(self, goal, obstacle):
        # Compute the attractive potential
        U_att = self.compute_attractive_potential(goal)

        # Compute the repulsive potential
        U_rep = self.compute_repulsive_potential(obstacle)

        # Compute the total potential
        U = U_att + U_rep

        return U

    def move(self, goal, obstacle):
        # Compute the gradient of the potential field
        gradient_x = (self.compute_potential_field(goal, obstacle) -
                      self.compute_potential_field(self.x - 0.1, self.y), obstacle)/0.1
        gradient_y = (self.compute_potential_field(goal, obstacle) -
                      self.compute_potential_field(self.x, self.y - 0.1), obstacle)/0.1

        # Compute the angle of the gradient
        theta = math.atan2(gradient_y, gradient_x)

        # Set the linear and angular velocities
        linear_vel = 0.1
        angular_vel = 0.1*math.sin(theta)

        # Create and publish the velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.set_speed_angle(linear_vel, angular_vel)

        return cmd_vel.linear.x, cmd_vel.angular.z


def run_demo():
    """Main loop"""
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")

    # Timing
    rospy.sleep(5+int(robot_name[-1]))
    while not rospy.is_shutdown():
        # Strategy
        goalred = Point()
        goalred.x = -21.21320344
        goalred.y = 21.21320344
        goalred.z = 0

        goalgreen = Point()
        goalgreen.x = 21.21320344
        goalgreen.y = 21.21320344
        goalgreen.z = 0

        goalblue = Point()
        goalblue.x = 0
        goalblue.y = -30
        goalblue.z = 0

        obs = Point()
        obs.x = 0
        obs.y = 10
        obs.z = 0

        if(int(robot_name == 1)):
            vel, ang = robot.move(goalred, obs)
        elif (int(robot_name == 2)):
            vel, ang = robot.move(goalgreen, obs)
        elif (int(robot_name == 3)):
            vel, ang = robot.move(goalblue, obs)            

        distance = float(robot.getDistanceToFlag())
        print(f"{robot_name} distance to flag = ", distance)

        # Finishing by publishing the desired speed.
        # DO NOT TOUCH.

        robot.set_speed_angle(vel, ang)
        rospy.sleep(0.5)


if __name__ == "__main__":

    print("Running ROS..")
    rospy.init_node("Controller", anonymous=True)
    run_demo()
