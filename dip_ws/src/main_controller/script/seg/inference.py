import cv2
import numpy as np
from config import *

def infer(src_img, Min_Area=500):
    crop_size = [500, 500]
    img = resize_keep_aspectratio(src_img, crop_size)
    src_w, src_h, _ = src_img.shape
    resize_w, resize_h, _ = img.shape
    scale_w = src_w / resize_w
    scale_h = src_h / resize_h
    _, red_contours = predict(img, red)
    red_contours = choose_plate(red_contours, Min_Area)
    red_center = draw_contours(src_img, red_contours, scale_w, scale_h)
    _, black_contours = predict(img, black)
    black_contours = choose_plate(black_contours, Min_Area)
    black_center = draw_contours(src_img, black_contours, scale_w, scale_h)

    return red_center, black_center

def resize_keep_aspectratio(image_src, dst_size):
    src_h, src_w = image_src.shape[:2]
    dst_h, dst_w = dst_size
    h = dst_w * (float(src_h) / src_w)  # 按照ｗ做等比缩放
    w = dst_h * (float(src_w) / src_h)  # 按照h做等比缩放
    h = int(h)
    w = int(w)

    if h <= dst_h:
        image_dst = cv2.resize(image_src, (dst_w, int(h)))
    else:
        image_dst = cv2.resize(image_src, (int(w), dst_h))
    h_, w_ = image_dst.shape[:2]

    return image_dst

def resize_photo(imgArr, MAX_WIDTH=1000):
    img = imgArr
    rows, cols = img.shape[:2]  # 获取输入图像的高和宽
    # 如果宽度大于设定的阈值
    if cols > MAX_WIDTH:
        change_rate = MAX_WIDTH / cols
        img = cv2.resize(img, (MAX_WIDTH, int(rows * change_rate)), interpolation=cv2.INTER_AREA)
    return img

def hsv_color_find(img, hsv_range):
    img_copy = img.copy()
    hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)
    # low_hsv = np.array([0, 43, 46])
    # high_hsv = np.array([13, 255, 255])
    low_hsv, high_hsv = hsv_range[0], hsv_range[1]
    # 设置HSV的阈值
    mask = cv2.inRange(hsv, lowerb=low_hsv, upperb=high_hsv)
    cv2.imshow("hsv_color", mask)
    res = cv2.bitwise_and(img_copy, img_copy, mask=mask)

    return res

def predict(imageArr, hsv):
    img_copy = imageArr.copy()
    img_copy = hsv_color_find(img_copy, hsv)
    gray_img = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    gray_img_ = cv2.GaussianBlur(gray_img, (5, 5), 0, 0, cv2.BORDER_DEFAULT)
    kernel = np.ones((30, 30), np.uint8)
    img_opening = cv2.morphologyEx(gray_img, cv2.MORPH_OPEN, kernel)
    img_opening = cv2.addWeighted(gray_img, 1, img_opening, -1, 0)
    ret2, img_thresh = cv2.threshold(img_opening, 0, 255, cv2.THRESH_BINARY)
    img_edge = cv2.morphologyEx(img_thresh, cv2.MORPH_CLOSE, kernel)
    img_edge = cv2.morphologyEx(img_edge, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy = cv2.findContours(img_edge, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    return gray_img_, contours

def draw_contours(img, contours, scale_w, scale_h):
    center = []
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        cv2.rectangle(img, (int(x*scale_w), int(y*scale_h)), (int((x + w)*scale_w), int((y + h)*scale_h)), (0, 255, 0), 2)
        center.append([int((x+w/2)*scale_w), int((y+h/2)*scale_h)])
    cv2.imshow("contours", img)
    return center

def choose_plate(contours, Min_Area=500):
    temp_contours = []
    for contour in contours:
        if cv2.contourArea(contour) > Min_Area:
            temp_contours.append(contour)
    plate = []
    plate1 = []
    for temp_contour in temp_contours:
        rect_tupple = cv2.minAreaRect(temp_contour)
        rect_width, rect_height = rect_tupple[1]
        if rect_width > rect_height:
            rect_width, rect_height = rect_height, rect_width
        aspect_ratio = rect_width / rect_height
        if aspect_ratio > 0.2 and aspect_ratio < 0.7:
            plate.append(temp_contour)

    if len(plate) > 1:
        for temp_contour in plate:
            rect_tupple = cv2.minAreaRect(temp_contour)
            rect_width, rect_height = rect_tupple[1]
            if rect_width > rect_height:
                rect_width, rect_height = rect_height, rect_width
            aspect_ratio = rect_width / rect_height
            if aspect_ratio > 0.2 and aspect_ratio < 0.6:
                plate1.append(temp_contour)

    if len(plate1) > 0:
        return plate1
    return plate

if __name__ == "__main__":
    # 你要识别的图片
    img = cv2.imread('195.jpg')
    infer(img)
    cv2.waitKey()

