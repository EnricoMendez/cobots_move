#!/usr/bin/env python3 
import rospy
from xarm_msgs.srv import Move, MoveRequest
#This class will create a client to make the Uarm robot move to the given pose.
class UarmToolClient(): 
    def __init__(self): 
        print("Waiting for Move Line service")
        rospy.wait_for_service('/ufactory/move_line') #This is a convenience method that blocks until the service is available.
        print("Move Line service is ready, I will call it")
        #Next we create a handle for calling the service 
        move_joint_service_proxy = rospy.ServiceProxy('/ufactory/move_line', Move)
        my_move = MoveRequest()
        #To call Cartesian motion expressed in robot TOOL Coordinate, 
        # with max speed 200 mm/s and acceleration 2000 mm/s^2, 
        # the following will move a relative motion (delta_x=50mm, delta_y=50mm, delta_z=50mm)
        # along the current Tool coordinate, no orientation change:

        my_move.pose = [250.0,100.0,300.0,3.1416,0.0,0.0] # (delta_x[mm],delta_y[mm],delta_z[mm],roll[rad],pitch[rad],yaw[rad]) 
        my_move.mvvelo = 200 #200 mm/s
        my_move.mvacc = 2000 #[mm/s2]
        print("calling the service")
        resp = move_joint_service_proxy(my_move)
        print("The service was called")
        r = rospy.Rate(2)
        while not rospy.is_shutdown(): 
            print(resp)
            r.sleep()  #It is very important that the r.sleep function is called at least onece every cycle. 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    rospy.init_node("uarm_move_tool_client", anonymous=True) 
    UarmToolClient() 
