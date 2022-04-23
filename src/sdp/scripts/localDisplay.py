import cv2
import laneDetect

from f1tenth_simulator.srv import *
import rospy

INPUT_FRAMERATE = 10
def gstreamer_pipeline(
    sensor_id = 0,
    capture_width = 1920,
    capture_height = 1080,
    display_width = 640,
    display_height = 360,
    framerate = INPUT_FRAMERATE,
    flip_method = 2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert !"
        "video/x-raw, format=(string)BGR ! queue max-size-buffers=1 leaky=downstream ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
def show_camera(): 
    window_title = "CSI Camera"

    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True: 
                ret_val, frame = video_capture.read()
                #if ret_val:

                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    
                    dat = laneDetect.modelTest2(frame)
                    if dat[0]:
                        angle = dat[0]
                        servo_control_client(int(angle), 15)
                    cv2.imshow(window_title, dat[1])
                else: 
                    break
                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27 or keyCode == ord('q'): 
                    break
        finally: 
            video_capture.release()
            cv2.destroyAllWindows()
    else: 
        print("Error: Unable to open camera")

def servo_control_client(a, ch):
    rospy.wait_for_service('servo_control_srv')
    try:
        servo_control_req = rospy.ServiceProxy('servo_control_srv', ServoData)
        # servo_control_req = f1tenth_simulator.srv.
        servo_control_resp = servo_control_req(angle=a)
        return servo_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    show_camera()
