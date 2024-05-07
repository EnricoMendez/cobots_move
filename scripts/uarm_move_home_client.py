#!/usr/bin/env python3
import rospy
from xarm_msgs.srv import Move, MoveRequest
#This class will create a client to make the Uarm robot move to the home pose.
class UarmHomeClient(): 
    def __init__(self): 
        print("Waiting for go home service")
        rospy.wait_for_service('/ufactory/go_home') #This is a convenience method that blocks until the service is available.
        print("Go home service is ready, I will call it")
        #Next we create a handle for calling the service 
        go_home_service_proxy = rospy.ServiceProxy('/ufactory/go_home', Move)
        my_move = MoveRequest()
        my_move.pose = []
        my_move.mvvelo = 0.35 #0.35 rad/s
        my_move.mvacc = 7.0 #[rad/s2]
        print("calling the service")
        resp = go_home_service_proxy(my_move)
        print("The service was called")
        r = rospy.Rate(2)
        while not rospy.is_shutdown(): 
            print(resp)
            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("uarm_move_home_client", anonymous=True) 
    UarmHomeClient() 
