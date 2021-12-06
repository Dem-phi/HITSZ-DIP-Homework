#!/usr/bin/python
# -*- coding: utf-8 -*-
from inference import Inference
import pyzed.sl as sl
import cv2
import rospy
from std_msgs.msg import Int32MultiArray  #消息头文件
import time

crop_size = [640, 640]
k_top = 2
threshold = 0.1
k_ratio = 0.3

if __name__ == '__main__':
    # init ros
    pub = rospy.Publisher('/chatter', Int32MultiArray, queue_size=10)
    rospy.init_node('paddle_ros', anonymous=True)
    rate = rospy.Rate(10)
    infer = Inference(crop_size=crop_size, k_top=k_top)
    # init zed
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.sdk_verbose = True
    err = zed.open(init_params)
    if (err != sl.ERROR_CODE.SUCCESS):
        exit(-1)
    image = sl.Mat()
    depth = sl.Mat()
    # infer
    while not rospy.is_shutdown():
        if (zed.grab() == sl.ERROR_CODE.SUCCESS):
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        image_value = image.get_data()
        depth_value = depth.get_data()
        frame = image_value[:, :int(image_value.shape[1]/2), :-1]
        h, w, _ = frame.shape
        pass_time = time.time()
        output = infer.predict(frame)
        print(1/(time.time()-pass_time))
        if len(output) != 0:
            if output[0][1] > threshold:
                for i in range(len(output)):
                    print("accurate: "+str(output[i][1]))
                    if output[i][1] > threshold:
                        cv2.rectangle(frame, (int(output[i][2]/crop_size[0]*w), int(output[i][3]/crop_size[1]*h)),
                                      (int(output[i][4]/crop_size[0]*w), int(output[i][5]/crop_size[1]*h)), [0, 0, 255], thickness=2)
                        cv2.rectangle(depth_value, (int(output[i][2] / crop_size[0] * w), int(output[i][3] / crop_size[1] * h)),
                                      (int(output[i][4] / crop_size[0] * w), int(output[i][5] / crop_size[1] * h)), [0, 0, 255], thickness=2)
                        center_x = int((output[i][2] + output[i][4]) / crop_size[0] * w / 2)
                        center_y = int((output[i][3] + output[i][5]) / crop_size[1] * h / 2)
                        center_point = [int(output[i][0]), center_x, center_y, int(k_ratio*depth_value[center_x][center_y])]
                        center_msg = Int32MultiArray(data=center_point)
                        rospy.loginfo(center_msg)
                        pub.publish(center_msg)
                        rate.sleep()
                    else:
                        pass
            else:
                pass
        cv2.imshow("frame", frame)
        cv2.imshow("depth", depth_value)
        cv2.waitKey(1)
