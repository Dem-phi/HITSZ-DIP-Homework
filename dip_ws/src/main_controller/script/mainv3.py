#!/usr/bin/python
# -*- coding: utf-8 -*-
from numpy.core.numeric import NaN
from inference import Inference
import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from std_msgs.msg import Int32MultiArray  #消息头文件
import time
import math

crop_size = [640, 640]
k_top = 2
threshold = 0.1

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
    init_params.depth_minimum_distance = 0.3
    err = zed.open(init_params)
    if (err != sl.ERROR_CODE.SUCCESS):
        exit(-1)
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    # infer
    while not rospy.is_shutdown():
        if (zed.grab() == sl.ERROR_CODE.SUCCESS):
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA) 
        image_value = np.array(image.get_data())
        depth_value = np.array(depth.get_data())
        frame = np.array(image_value[:, :, :-1])
        h, w, _ = frame.shape
        label_list = []
        center_x_list = []
        center_y_list = []
        distance_list = []

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
                        point_cloud_value = point_cloud.get_value(center_x, center_y)[1]
                        distance=math.sqrt(point_cloud_value[0]*point_cloud_value[0]+point_cloud_value[1]*point_cloud_value[1] + point_cloud_value[2]*point_cloud_value[2])
                        print("Distance to Camera at (", center_x, center_y, "): ", distance, "mm")
                        if center_x > 640:
                            label_list.append(0)
                        else:
                            label_list.append(1)
                        center_x_list.append(center_x)
                        center_y_list.append(center_y)
                        try:
                            distance = int(distance)
                        except:
                            distance_list.append(300)
                        else:
                            distance_list.append(int(distance))

                    else:
                        pass
            else:
                pass
        list_len = min(len(label_list), 4)
        center_point = []
        for i in range(list_len):
            center_point.append(label_list[i])
            center_point.append(center_x_list[i])
            center_point.append(center_y_list[i])
            center_point.append(distance_list[i])
        for i in range(4-list_len):
            center_point.extend([0, 0, 0, 0])
        center_msg = Int32MultiArray(data=center_point)
        rospy.loginfo(center_msg)
        pub.publish(center_msg)
        cv2.imshow("frame", frame)
        #cv2.imshow("depth", depth_value)
        cv2.waitKey(1)
        rate.sleep()
