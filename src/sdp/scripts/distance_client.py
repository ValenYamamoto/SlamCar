import rospy
from sdp.srv import *

def motor_control_client(x):
    rospy.wait_for_service('tof_readings')
    try:
        mc = rospy.ServiceProxy('motor_control_srv', MotorData)
        resp1 = mc(x)
        return resp1.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print("Starting Client...")
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        print(usage())
        sys.exit(1)
    print("%s = %s"%(x, motor_control_client(x)))
