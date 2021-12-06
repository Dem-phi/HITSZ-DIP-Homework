from inference import yolo
import pyzed.sl as sl
import rospy
from std_msgs.msg import Int32MultiArray  #消息头文件
import numpy as np
import math
import cv2

Net_config = [
    {'confThreshold': 0.5, 'nmsThreshold': 0.4, 'inpWidth': 320, 'inpHeight': 320, 'classesFile': 'smart.names',
     'modelConfiguration': 'yolov4-tiny/yolov4-tiny.cfg', 'modelWeights': 'yolov4-tiny/yolov4-tiny_last.weights',
     'netname': 'yolov4-tiny'}]

net_type = 0
yolo_net = yolo(Net_config[net_type])

if __name__ == '__main__':
    pub = rospy.Publisher('/chatter', Int32MultiArray, queue_size=10)
    rospy.init_node('paddle_ros', anonymous=True)
    rate = rospy.Rate(10)
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
        src_img, results = yolo_net.detect(frame)
        label_list = []
        center_x_list = []
        center_y_list = []
        distance_list = []
        if len(results) > 0:
            for result in results:
                point_cloud_value = point_cloud.get_value(result[1], result[2])[1]
                distance = math.sqrt(
                    point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1] +
                    point_cloud_value[2] * point_cloud_value[2])
                try:
                    distance = int(distance)
                except:
                    distance_list.append(300)
                else:
                    if distance < 5000:
                        distance_list.append(int(distance))
                        label_list.append(int(result[0]))
                        center_x_list.append(int(result[1]))
                        center_y_list.append(int(result[2]))

        list_len = min(len(label_list), 4)
        center_point = []
        for i in range(list_len):
            center_point.append(int(1-label_list[i]))
            center_point.append(center_x_list[i])
            center_point.append(center_y_list[i])
            center_point.append(distance_list[i])
        for i in range(4 - list_len):
            center_point.extend([0, 0, 0, 0])

        center_msg = Int32MultiArray(data=center_point)
        rospy.loginfo(center_msg)
        pub.publish(center_msg)

        cv2.imshow('video', src_img)
        cv2.waitKey(1)
        rate.sleep()
