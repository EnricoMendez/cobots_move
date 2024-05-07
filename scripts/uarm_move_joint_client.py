#!/usr/bin/env python3
import rospy
from xarm_msgs.srv import Move, MoveRequest
#This class will create a client to make the Uarm robot move to the given pose.
class UarmJointClient(): 
    def __init__(self): 
        print("Waiting for Move Joint service")
        rospy.wait_for_service('/ufactory/move_joint') #This is a convenience method that blocks until the service is available.
        print("Move Joint service is ready, I will call it")
        #Next we create a handle for calling the service 
        move_joint_service_proxy = rospy.ServiceProxy('/ufactory/move_joint', Move)
        my_move = MoveRequest()
        #Example move joint 3 to (45 deg-> pi/4=0.7853)
        my_move.pose = [0.0,0.0,0.7853,0.0,0.0,0.0] #angles for each joint in [rad]; 
        my_move.mvvelo = 0.35 #0.35 rad/s
        my_move.mvacc = 7.0 #[rad/s2]
        print("calling the service")
        resp = move_joint_service_proxy(my_move)
        print("The service was called")
        r = rospy.Rate(2)
        while not rospy.is_shutdown(): 
            print(resp)
            r.sleep()  #It is very important that the r.sleep function is called at least onece every cycle. 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("uarm_move_joint_client", anonymous=True) 
    UarmJointClient() 