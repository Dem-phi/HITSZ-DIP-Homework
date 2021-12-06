'''
Author: your name
Date: 2021-12-02 21:40:05
LastEditTime: 2021-12-02 23:02:21
LastEditors: Please set LastEditors
Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
FilePath: /main_controller/script/main.py
'''
#!/usr/bin/python
# -*- coding: utf-8 -*-
from inference import Inference
import cv2
import rospy
from std_msgs.msg import Int32MultiArray  #消息头文件
import time

crop_size = [640, 640]
k_top = 2
threshold = 0.1

if __name__ == '__main__':
    pub = rospy.Publisher('/chatter', Int32MultiArray, queue_size=10)
    rospy.init_node('paddle_ros', anonymous=True)
    rate = rospy.Rate(10)
    infer = Inference(crop_size=crop_size, k_top=k_top)
    capture = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        ret, frame = capture.read()
        frame = frame[:, :int(frame.shape[1]/2), :]
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
                        center_point = [i, int((output[i][2] + output[i][4]) / crop_size[0] * w / 2),
                                        int((output[i][3] + output[i][5]) / crop_size[1] * h / 2)]
                        center_msg = Int32MultiArray(data=center_point)
                        rospy.loginfo(center_msg)
                        pub.publish(center_msg)
                        rate.sleep()
                    else:
                        pass
            else:
                pass
        cv2.imshow("video", frame)
        cv2.waitKey(1)
