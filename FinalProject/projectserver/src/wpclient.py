#!/usr/bin/env python
 
import sys
import rospy
from projectserver.srv import *
 
def Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony):    
    rospy.wait_for_service('Final_ints')
    try:
        Final_ints = rospy.ServiceProxy('Final_ints', getwaypoint)

        resp1 = Final_ints(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
        return resp1
    except rospy.ServiceException as e:
        print ("Service call failed: %s"%e)

def usage():
    return "%s [firstwp symboldetected symbolcolor symbolpostionx symbolpositiony]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 6:
        #firstwp = bool(sys.argv[1])
        firstwp = sys.argv[1]
        if firstwp == 'True': firstwp = bool(True)
        if firstwp == 'False': firstwp = bool(False)
        symboldetected = sys.argv[2]
        if symboldetected == 'True': symboldetected = bool(True)
        if symboldetected == 'False': symboldetected = bool(False)
        #symboldetected = bool(sys.argv[2])
        symbolcolor = int(sys.argv[3])
        symbolpositionx = float(sys.argv[4])
        symbolpositiony = float(sys.argv[5])


    else:
        print (usage())
        sys.exit(1)
    result = Final_ints_client(firstwp,symboldetected,symbolcolor,symbolpositionx,symbolpositiony)
    print ("%s"%result)