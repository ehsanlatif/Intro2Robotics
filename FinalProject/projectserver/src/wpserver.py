#!/usr/bin/env python

from projectserver.srv import getwaypoint,getwaypointResponse
import rospy


# Color = [2,1,1,3,3]
# X =  [2,-1.5,-2.8,-2.7,-0.4]
# Y = [2,1,1.8,3.2,5.7]
Color = [1,2,2,0] # 1 is red, 2 is green
X =  [-2.5, 2.2, -0.2,0.0]
Y = [2.5, 7.0, -3.4,0.0]   

def handle_Final_ints(req):
    global i
    global X
    global Y
    if req.firstwp == True :
        i = 0 
        success = True
        waypointx = X[i]
        waypointy = Y[i]
        return getwaypointResponse(success,waypointx,waypointy)
    else:
        if req.symboldetected == True:
            print(req)
            # Also, in line 23, make the following changes appropriately indented in the code.
            if req.symbolcolor == Color[i] and round(req.symbolpositionx,1) >= round(X[i],1)-0.2 and round(req.symbolpositionx,1) <= round(X[i],1)+0.2 and round(req.symbolpositiony,1) >= round(Y[i],1)-0.2 and round(req.symbolpositiony,1) <= round(Y[i],1)+0.2:
            # if req.symbolcolor == Color[i] and req.symbolpositionx == X[i] and req.symbolpositiony == Y[i]:
                success = True
                i = (i+1)%4
            else:
                success = False
            waypointx = X[i]
            waypointy = Y[i] 
            return getwaypointResponse(success,waypointx,waypointy)
def Final_ints_server():
    global i 
    i = 0
    rospy.init_node('Final_ints_server')
    s = rospy.Service('Final_ints', getwaypoint, handle_Final_ints)
    print("Ready to go.")
    rospy.spin()

if __name__ == "__main__":
    Final_ints_server()
