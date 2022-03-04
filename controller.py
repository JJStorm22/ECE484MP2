import rospy
import math
from gazebo_msgs.srv import GetModelState, GetModelStateResponse
from gazebo_msgs.msg import ModelState
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from std_msgs.msg import Float32MultiArray
from util import euler_to_quaternion, quaternion_to_euler

class vehicleController():

    def __init__(self):
        # Publisher to publish the control input to the vehicle model
        self.controlPub = rospy.Publisher("/ackermann_cmd", AckermannDrive, queue_size = 1)

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = serviceResponse(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            resp = GetModelStateResponse()
            resp.success = False
        return resp

    def execute(self, currentPose, referencePose):
        # Compute the control input to the vehicle according to the 
        # current and reference pose of the vehicle
        # Input:
        #   currentPose: ModelState, the current state of the vehicle
        #   referencePose: list, the reference state of the vehicle, 
        #       the element in the list are [ref_x, ref_y, ref_theta, ref_v]
        # Output: None

        # TODO: Implement this function
        current_x = currentPose.pose.position.x
        current_y = currentPose.pose.position.y
        current_theta = (quaternion_to_euler(currentPose.pose.orientation.x, currentPose.pose.orientation.y, currentPose.pose.orientation.z, currentPose.pose.orientation.w)[2])#*(math.pi/180)
        current_velocity = np.linalg.norm([currentPose.twist.linear.x,currentPose.twist.linear.y],2)

        error_x = ((np.cos(referencePose[2]))*(referencePose[0] - current_x))+(np.cos(referencePose[2]))*(referencePose[1]-current_y)
        error_y = (-(np.sin(referencePose[2]))*(referencePose[0] - current_x))+(np.cos(referencePose[2]))*(referencePose[1]-current_y)
        error_theta = referencePose[2] - current_theta
        error_velocity = referencePose[3] - current_velocity

        # kx = 1.0 3rd turn
        # ky = 0.1
        # kv = 1.5
        # k0 = 2.0

        kx = 0.1 
        ky = 0.05
        kv = 0.5
        k0 = 2.0

        # kx = 0.09 
        # ky = 0.09
        # kv = 1
        # k0 = 2.0

        # kx = 1.0
        # ky = 0.25
        # kv = 1.0
        # k0 = 1.5

        K = np.array([[kx, 0, 0, kv],[0, ky, k0, 0]])
        delta = np.array([[error_x],[error_y],[error_theta],[error_velocity]])

        u = np.matmul(K,delta)
        print(u)
        #Pack computed velocity and steering angle into Ackermann command
        # newAckermannCmd = AckermannDrive(u)
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = u[0][0]
        newAckermannCmd.steering_angle = u[1][0]

        # Publish the computed control input to vehicle model
        self.controlPub.publish(newAckermannCmd)


    def setModelState(self, currState, targetState, vehicle_state = "run"):
        control = self.rearWheelFeedback(currState, targetState)
        self.controlPub.publish(control)

    def stop(self):
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.speed = 0
        self.controlPub.publish(newAckermannCmd)
