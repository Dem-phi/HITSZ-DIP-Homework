from inference import infer
import pyzed.sl as sl
import rospy
from std_msgs.msg import Int32MultiArray  #消息头文件
import numpy as np
import math
import cv2

if __name__ == '__main__':
    # init ros
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
        h, w, _ = frame.shape

        label_list = []
        center_x_list = []
        center_y_list = []
        distance_list = []

        red_center, black_center = infer(frame)

        list_len = min(2, len(red_center))
        for i in range(list_len):
            point_cloud_value = point_cloud.get_value(red_center[i][0], red_center[i][1])[1]
            distance = math.sqrt(
                point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1] +
                point_cloud_value[2] * point_cloud_value[2])
            print("Distance to Camera at (", red_center[i][0], red_center[i][1], "): ", distance, "mm")
            label_list.append(0)
            center_x_list.append(red_center[i][0])
            center_y_list.append(red_center[i][1])
            try:
                distance = int(distance)
            except:
                distance_list.append(300)
            else:
                distance_list.append(int(distance))

        list_len = min(2, len(black_center))
        for i in range(list_len):
            point_cloud_value = point_cloud.get_value(black_center[i][0], black_center[i][1])[1]
            distance = math.sqrt(
                point_cloud_value[0] * point_cloud_value[0] + point_cloud_value[1] * point_cloud_value[1] +
                point_cloud_value[2] * point_cloud_value[2])
            print("Distance to Camera at (", black_center[i][0], black_center[i][1], "): ", distance, "mm")
            label_list.append(1)
            center_x_list.append(black_center[i][0])
            center_y_list.append(black_center[i][1])
            try:
                distance = int(distance)
            except:
                distance_list.append(300)
            else:
                distance_list.append(int(distance))

        list_len = min(len(label_list), 4)
        center_point = []
        for i in range(list_len):
            center_point.append(label_list[i])
            center_point.append(center_x_list[i])
            center_point.append(center_y_list[i])
            center_point.append(distance_list[i])
        for i in range(4 - list_len):
            center_point.extend([0, 0, 0, 0])

        center_msg = Int32MultiArray(data=center_point)
        rospy.loginfo(center_msg)
        pub.publish(center_msg)
        cv2.imshow("frame", frame)
        cv2.imshow("depth", depth_value)
        cv2.waitKey(1)
        rate.sleep()
