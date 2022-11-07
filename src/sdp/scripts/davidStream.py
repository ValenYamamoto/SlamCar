import cv2
import gi
import laneDetect 
import numpy as np
import sys
from sdp.srv import *
import rospy
import gpu_instance

INPUT_FRAMERATE = 10 #This gets used by gst preprocessing and the rtsp server 

if len(sys.argv) > 1:
    fm = int(sys.argv[1])
    if fm <= 30 and fm > 0:
        print("Valid Framerate passed" + str(fm))
        
        INPUT_FRAMERATE = fm
    else:
        print("Invalid Framerate, defaulting to: " + str(INPUT_FRAMERATE))
else:
    print("No Framerate passed  defaulting to: " + str(INPUT_FRAMERATE))


gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GObject

window_name = "CSI Camera"

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

    print(gstreamer_pipeline(flip_method=2))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                #cv2.putText(frame, "OOOGA BOOOOOGA", (5,40), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 3)
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    pass
                    cv2.imshow(window_title, frame)
                    #out.write(frame)
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

# see: https://stackoverflow.com/questions/47396372/write-opencv-frames-into-gstreamer-rtsp-server-pipeline?rq=1
class SensorFactory(GstRtspServer.RTSPMediaFactory):
    def __init__(self, **properties):
        super(SensorFactory, self).__init__(**properties)
        self.cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2))
        self.number_frames = 0
        self.fps = INPUT_FRAMERATE 
        self.duration = 1 / self.fps * Gst.SECOND  # duration of a frame in nanoseconds
        #Original launch from david 
        #self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                             #'caps=video/x-raw,format=BGR,width=640,height=480,framerate={}/1 ' \
                             #'! videoconvert ! video/x-raw,format=I420 ' \
                             #'! x264enc speed-preset=ultrafast tune=zerolatency ' \
                             #'! rtph264pay config-interval=1 name=pay0 pt=96'.format(self.fps) 

        self.launch_string = 'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME ' \
                             'caps=video/x-raw,format=BGR,width=640,height=360,framerate={}/1 ' \
                             '! videoconvert ! video/x-raw,format=I420 ' \
                             '! x264enc speed-preset=ultrafast tune=zerolatency ' \
                             '! rtph264pay config-interval=1 name=pay0 pt=96'.format(self.fps) 

    def on_need_data(self, src, lenght):
        if self.cap.isOpened():
            ret, frame = self.cap.read() 
            if ret:  
                #cv2.putText(frame, "OOOGA BOOOOOGA", (5,40), cv2.FONT_HERSHEY_PLAIN, 3, (0,0,255), 3)
                #avg_lines = laneDetect.extractLinesFromImage(frame)
                #print("avg_lines: " + str(avg_lines))
                #if(1):
                #    frame = laneDetect.applyLinesToImage(frame, avg_lines)
                #    #frame = np.bitwise_or(frame, avg_lines[:,:,np.newaxis])
                #else:
                #    print("No Hough lines found")
                #    #Just stream the original frame
                dat = laneDetect.modelTest2(frame)
                print(dat[0])
                if dat[0]:
                    angle = dat[0]
                    servo_control_client(int(angle), 15)
                frame = dat[1] 
                data = frame.tostring()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.duration = self.duration
                timestamp = self.number_frames * self.duration
                buf.pts = buf.dts = int(timestamp)
                buf.offset = timestamp
                self.number_frames += 1
                retval = src.emit('push-buffer', buf)
                print("frames" + str(self.number_frames))
                #print('pushed buffer, frame {}, duration {} ns, durations {} s'.format(self.number_frames,
                #                                                                       self.duration,
                #      BGR,wi                                                                 self.duration / Gst.SECOND))
                if retval != Gst.FlowReturn.OK:
                    print(retval)

    def do_create_element(self, url):
        return Gst.parse_launch(self.launch_string)

    def do_configure(self, rtsp_media):
        self.number_frames = 0
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)


class GstServer(GstRtspServer.RTSPServer):
    def __init__(self, **properties):
        super(GstServer, self).__init__(**properties)
        self.factory = SensorFactory()
        self.factory.set_shared(True)
        self.get_mount_points().add_factory("/test", self.factory)
        self.attach(None)

def servo_control_client(a, ch):
    rospy.wait_for_service('servo_control_srv')
    try:
        servo_control_req = rospy.ServiceProxy('servo_control_srv', ServoData)
        # servo_control_req = f1tenth_simulator.srv.
        servo_control_resp = servo_control_req(angle=a)
        return servo_control_resp.error
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#show_camera()
GObject.threads_init()
Gst.init(None)

server = GstServer()

loop = GObject.MainLoop()
loop.run()


