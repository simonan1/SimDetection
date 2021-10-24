import socket
import struct  
import threading
import cv2 as cv
# sys.path.append("path/foo/bar/")
from objectDetectionSimona import *
import time

# v4l2-ctl --list-devices

CAMERA = cv.VideoCapture(0)   # 0 -> index of camera
CUR_IMG = []
IMG_ADDRS = ("192.168.43.6",50003)
AZ_ADDRS = ("192.168.43.6",50004)
IMG_ARR = []
X_REF=Y_REF = 0
KEY_REF = []
DES_REF = []
TIME_PREV_SEND = 0
TIME_PREV_GRAB = 0

img_ind=0
resize_ratio = 2

def sendAzErrViaUdp(az_deg):
    global TIME_PREV_SEND
    cur_time = time.time()
    print("In sendAzErrViaUdp az err = ",az_deg, "dt grab  = ",cur_time - TIME_PREV_SEND)
    TIME_PREV_SEND = cur_time
    sender_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    trkErrDeg = np.array([az_deg], dtype='f') 
    x = sender_socket.sendto(trkErrDeg, AZ_ADDRS)

def sendImgViaUdp(img):
    global resize_ratio
    # print("In send Img")
    sender_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    # ratio = 2
    w = int(640/resize_ratio)
    h = int(480/resize_ratio)
    img_red = cv.resize(img, (w, h), interpolation=cv.INTER_AREA)

    retval, commJpgBuf = cv.imencode('.jpg', img_red)  # retval, commJpgBuf = cv.imencode('.jpg', scene_red, (cv.IMWRITE_JPEG_QUALITY, 80))
    # cv.imshow("scene_red",scene_red) # cv.waitKey(1)
    x = sender_socket.sendto(commJpgBuf, IMG_ADDRS)
    print("------------ img data sent via udp --------------",len(commJpgBuf),img_red.shape)

    #You can not send messages (datagrams) larger than 2^16 65536 octets with UDP. The length field of a UDP packet is 16 bits. 
    
def capture_thread():
    reciever = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reciever.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    localhost = ("192.168.43.5",50001) #hostname -local ip for communication inside same computer,127.0.0.1 >port anything > 50000 (there are 2^16 ports)
    reciever.bind(localhost)
    print("In capture thread")
    try:
        thread = threading.Thread( target = capture_tr,args=["captureThread",localhost,reciever]) #threadName ,addr , sckt
        thread.start()
    except:
        print("Error: unable to start thread")
    return thread


def track_thread():
    reciever = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reciever.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    localhost = ("192.168.43.5",50002) #hostname -local ip for communication inside same computer,127.0.0.1 >port anything > 50000 (there are 2^16 ports)
    reciever.bind(localhost)
    print("In track_thread")
    try:
        thread = threading.Thread( target = track_tr,args=["track_thread",localhost,reciever]) #threadName ,addr , sckt
        thread.start()
    except:
        print("Error: unable to start thread")
    return thread

def cam_read_thread():
    print("In capture thread")
    try:
        thread = threading.Thread( target = cam_read_tr,args=["cam_thread"]) #threadName 
        thread.start()
    except:
        print("Error: unable to start thread")
    return thread

def cam_read_tr(threadName):
    global CAMERA
    global CUR_IMG
    global TIME_PREV_SEND
    global TIME_PREV_GRAB
    REQUIRED_DT = 0.1 #[sec]
    global KEY_REF
    global DES_REF
    global X_REF
    global Y_REF
    print(threadName)
    while True:
        s, img = CAMERA.read()
        if s:    # frame captured without any errors
            CUR_IMG = img
            cur_time = time.time()
            # print("dt grab  = ",cur_time - TIME_PREV_GRAB)
            # TIME_PREV_GRAB = cur_time
            # if (X_REF!=0 and len(DES_REF)>0 and len(KEY_REF)>0 and cur_time - TIME_PREV_SEND > REQUIRED_DT): # do detection and send err
            #     print("dt = ",cur_time - TIME_PREV_SEND,"X_REF = ", X_REF)
            #     TIME_PREV_SEND = cur_time
            #     print("perform detection from cam_read_tr. len(DES_REF) = ", len(DES_REF))
            #     az_deg,scene = getTrackErr(KEY_REF, DES_REF,X_REF,Y_REF)
            #     if (az_deg!=999):
            #         print("Send from cam_read_tr")
            #         sendImgViaUdp(scene)
            #         sendAzErrViaUdp(az_deg)
            #     else:
            #         print("no detection")
        else:
            print("----- FAIL CAMERA READ -----")

def capture_tr(threadName ,addr , sckt):
    global CAMERA
    global CUR_IMG
    global IMG_ARR
    global img_ind
    global KEY_REF
    global DES_REF

    print(threadName)

    while True:
        data, addr = sckt.recvfrom(1024) # kilobyte ? the number of bytes to be read from the UDP socket. 
        # s0, img0 = CAMERA.read()
        # s1, img1 = CAMERA.read()
        # s2, img2 = CAMERA.read()
        # s3, img3 = CAMERA.read()
        # s, img = CAMERA.read()
        img = CUR_IMG
        s=True
        if s:    # frame captured without any errors
            if X_REF==0:
                print("Send from capture_tr")
                sendImgViaUdp(img)
                img_ind = img_ind+1
                filename = "kobi"+str(img_ind)+".jpg"
                IMG_ARR.append(img)
                cv.imwrite(filename,img)
            else:
                # print("perform detection from capture_tr")
                if (X_REF!=0 and len(DES_REF)>0 and len(KEY_REF)>0): # do detection and send err
                    print("perform detection from capture_tr")
                    az_deg,scene = getTrackErr(KEY_REF, DES_REF,X_REF,Y_REF)
                    if (az_deg!=999):
                        print("Send from capture_tr")
                        sendImgViaUdp(scene)
                        sendAzErrViaUdp(az_deg)
                else:
                    print("no detection")
                # az_deg,scene = getTrackErr(KEY_REF, DES_REF,X_REF,Y_REF)
                # if (az_deg!=999):
                #     print("Send from capture_tr")
                #     sendImgViaUdp(scene)
                #     sendAzErrViaUdp(az_deg)
                # else:
                #     print("no detection")

def getTrackErr(kp_r, d_r,ref_x,ref_y):
    global CAMERA
    global CUR_IMG
    az_deg = 999
    scene = cv.cvtColor(CUR_IMG, cv.COLOR_BGR2GRAY)
    # print("In getTrackErr scene size = ",scene.shape)

    kp_s, d_s = extractFeaturesDescriptions(scene)
    n_match, n_best, best_matches = matchTwoImagesKnn(d_s,d_r)
    if n_best > MIN_MATCH_COUNT:
        H,n_inliers,match_list = computeHomography(best_matches, kp_s, kp_r)
        print("n_inliers = ",n_inliers)
        in_text = "".join(['hg inliers num =',str(n_inliers)])
        scene = cv.putText(scene, in_text, (50, 110), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)
        scene = cv.drawKeypoints(scene, match_list, None, color=(0,255,255), flags=cv.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)	
        if n_inliers>=6:
            tgt = computePoint(H,ref_x,ref_y)
            az_deg = pixToAzimuth(int(tgt[0][0][0]),HFOV,HRES) 
            # print("Orange circle:",tgt[0][0][0],", ", tgt[0][0][1]) 
            # scene = cv.circle(scene, (int(tgt[0][0][0]), int(tgt[0][0][1])), radius=20, color=(0, 150, 255), thickness=10)
            scene = cv.circle(scene, (int(tgt[0][0][0]), int(tgt[0][0][1])), radius=5, color=(0, 150, 255), thickness=-1)
            # scene = cv.circle(scene, (int(ref_x), int(ref_y)), radius=5, color=(150, 0, 255), thickness=-1)
            az_range_text =  "".join(['Err = ',str(round(az_deg,1)), ' [deg]'])
            scene = cv.putText(scene, az_range_text, (50, 80), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA)
        else:
            scene = cv.putText(scene, "Homography not valid", (50, 80), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
    else:
        scene = cv.putText(scene, "target not detected", (50, 80), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv.LINE_AA)
    return az_deg,scene


def track_tr(threadName ,addr , sckt):
    global IMG_ARR
    global resize_ratio
    global X_REF
    global Y_REF
    global KEY_REF
    global DES_REF

    print(threadName)
    str = struct.Struct('qqq') # 3 long-long (index,x,y)

    while True:
        data, addr = sckt.recvfrom(1024) # kilobyte ? the number of bytes to be read from the UDP socket.
        packed_data = str.unpack(data)
        index = packed_data[0]
        x = packed_data[1]
        y = packed_data[2]
        print('index =', index, ' x =', x, ' y =', y)
        # print(IMG_ARR[int(index-1)].shape)
        scene = IMG_ARR[int(index-1)]
        X_REF = x*resize_ratio
        Y_REF = y*resize_ratio
        # scene_with_target = cv.circle(scene, (X_REF,Y_REF), radius=10, color=(0, 150, 255), thickness=-1)
        # cv.imshow("Win",scene_with_target)
        # cv.waitKey(0)
        ref_gray = cv.cvtColor(scene, cv.COLOR_BGR2GRAY)
        KEY_REF, DES_REF = extractFeaturesDescriptions(ref_gray)
        print("++++++++++++++++++ After extractFeaturesDescriptions ++++++++++++++++++++++++++")
        # az_deg,scene = getTrackErr(KEY_REF, DES_REF,X_REF,Y_REF)
        # if (az_deg!=999):
        #     print("Send from track_tr")
        #     sendImgViaUdp(scene)
        #     sendAzErrViaUdp(az_deg)
        # else:
        #     print("no detection")
             

def main():
    global CAMERA
    # initialize the camera
    print("begin")
    cam_read_thread()
    capture_thread()
    track_thread()
if __name__ == "__main__":
    		main()