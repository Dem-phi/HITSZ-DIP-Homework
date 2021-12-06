import cv2
import numpy as np
import time
# import rospy


class yolo:
    def __init__(self, config):
        print('Net use', config['netname'])
        self.confThreshold = config['confThreshold']
        self.nmsThreshold = config['nmsThreshold']
        self.inpWidth = config['inpWidth']
        self.inpHeight = config['inpHeight']
        with open(config['classesFile'], 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')
        self.colors = [np.random.randint(0, 255, size=3).tolist() for _ in range(len(self.classes))]
        self.net = cv2.dnn.readNet(config['modelConfiguration'], config['modelWeights'])
        self.stop = False
        self.ever_found = False

    def drawPred(self, frame, classId, conf, left, top, right, bottom):
        # Draw a bounding box.
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=3)
        label = '%.2f' % conf
        label = '%s:%s' % (self.classes[classId], label)
        # Display the label at the top of the bounding box
        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), thickness=2)
        return frame

    # Remove the bounding boxes with low confidence using non-maxima suppression
    def postprocess(self, frame, outs):
        frameHeight = frame.shape[0]
        frameWidth = frame.shape[1]
        classIds = []
        confidences = []
        boxes = []
        results = []
        centerxy_0 = []
        centerxy_1 = []
        centerx_0 = []
        centerx_1 = []
        centery_1 = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    center_x = int(detection[0] * frameWidth)
                    center_y = int(detection[1] * frameHeight)
                    width = int(detection[2] * frameWidth)
                    height = int(detection[3] * frameHeight)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    classIds.append(classId)
                    confidences.append(float(confidence))
                    boxes.append([left, top, width, height])
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.confThreshold, self.nmsThreshold)
        for i in indices:
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            self.drawPred(frame, classIds[i], confidences[i], left, top, left + width, top + height)
            center_x = left + round(width / 2)
            center_y = top + round(height / 2)
            results.append([classIds[i], center_x, center_y])

        return results

    def color_space(self, img):
        blur = cv2.GaussianBlur(img, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        low_red = np.array([0, 43, 46])
        high_red = np.array([13, 255, 255])
        mask = cv2.inRange(hsv, low_red, high_red)
        kernelSize = [(3, 3), (5, 5), (7, 7)]
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernelSize[2])
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, hierarchy = cv2.findContours(opening, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        length = len(contours)
        opening = cv2.cvtColor(opening, cv2.COLOR_GRAY2BGR)
        max_area = 0
        area=[]
        for i in range(length):
            cnt = contours[i]
            area.append(cv2.contourArea(cnt))
            [x, y, w, h] = cv2.boundingRect(cnt)
            cv2.rectangle(opening,(x,y),(x+w,y+h),(0,255,0),2)
        if max_area > 0.28 * img.shape[0] * img.shape[1]:
            self.stop = True
            self.turn_mode = False
        elif max_area > 0.12 * img.shape[0] * img.shape[1]:
            self.turn_mode = True
            self.stop = False
        else:
            self.stop = False
            self.turn_mode = False

    def detect(self, src_img):
        blob = cv2.dnn.blobFromImage(src_img, 1 / 255.0, (self.inpWidth, self.inpHeight), [0, 0, 0], swapRB=True,
                                     crop=False)
        # Sets the input to the network
        self.net.setInput(blob)
        # Runs the forward pass to get output of the output layers
        outs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        result = self.postprocess(src_img, outs)

        return src_img, result


Net_config = [
    {'confThreshold': 0.5, 'nmsThreshold': 0.4, 'inpWidth': 320, 'inpHeight': 320, 'classesFile': 'smart.names',
     'modelConfiguration': 'yolov4-tiny/yolov4-tiny.cfg', 'modelWeights': 'yolov4-tiny/yolov4-tiny_last.weights',
     'netname': 'yolov4-tiny'}]

if __name__ == "__main__":
    cap = cv2.VideoCapture(1)
    net_type = 0
    yolo_net = yolo(Net_config[net_type])
    ret = 1
    while True:
        time1 = time.time()
        frame = cv2.imread('24.jpg')
        if ret == 1:
            # size = (640, 500)
            # frame = cv2.resize(frame, size)
            src_img, result = yolo_net.detect(frame)
            time2 = time.time()
            fps = 1 / (time2 - time1)
            winName = 'video'
            cv2.namedWindow(winName, 0)
            cv2.resizeWindow(winName,640,640)
            cv2.moveWindow(winName, 700, 10)
            cv2.imshow(winName, src_img)
            cv2.waitKey(10)
        else:
            exit()
