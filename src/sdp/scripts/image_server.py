from flask import Response, Flask
import numpy as np
import cv2
import jetson.utils
import sys


app = Flask( __name__ )

def encode_video():
    while True:
        image = cv2.imread('/home/sdp10/catkin_ws/src/sdp/scripts/status_out.png', 0)
        if image is None:
            continue
        return_key, encoded = cv2.imencode( ".jpg", image )
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +  encoded.tobytes()  + b'\r\n\r\n' )

@app.route( "/" )
def streamFrames():
    return Response( encode_video(), mimetype="multipart/x-mixed-replace; boundary=frame" )

if __name__ == "__main__":
    app.run( "0.0.0.0", port="8000" )
    
