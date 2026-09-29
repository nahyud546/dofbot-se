# !/usr/bin/env python
# coding: utf-8
import cv2 as cv
import numpy as np
import math

class snake_target:
    def __init__(self):
        self.image = None
        self.cur_joint = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.Posture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def Image_Processing(self, img):
        '''
        Morphological transformation to remove small interference factors
        形态学变换去出细小的干扰因素
        :param img: 输入初始图像      Enter the initial image
        :return: 检测的轮廓点集(坐标)  Detected contour point set (coordinates)
        '''
        # Convert image to grayscale
        # 将图像转为灰度图
        gray_img = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
        # Get structuring elements of different shapes
        # 获取不同形状的结构元素
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        # morphological closure
        # 形态学闭操作
        dst_img = cv.morphologyEx(gray_img, cv.MORPH_CLOSE, kernel)
        # 开操作去除小噪点  opening operation to remove small noise
        dst_img = cv.morphologyEx(dst_img, cv.MORPH_OPEN, kernel)
        # Image Binarization Operation
        # 图像二值化操作
        ret, binary = cv.threshold(dst_img, 10, 255, cv.THRESH_BINARY)
        # 高斯模糊减少噪点  Gaussian blur to reduce noise
        binary = cv.GaussianBlur(binary, (5, 5), 0)
        # Get the set of contour points (coordinates)
        # 获取轮廓点集(坐标)
        find_contours = cv.findContours(binary, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        return contours

    def get_area(self, hsv_name, hsv_range):
        (lowerb, upperb) = hsv_range
        # Convert image to HSV
        # 将图像转换为HSV
        hsv_img = cv.cvtColor(self.image, cv.COLOR_BGR2HSV)
        # filter out elements between two arrays
        # 筛选出位于两个数组之间的元素
        color_mask = cv.inRange(hsv_img, lowerb, upperb)
        
        # 直接在HSV掩码上进行形态学操作，去除噪点
        # Perform morphological operations directly on HSV mask to remove noise
        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        color_mask = cv.morphologyEx(color_mask, cv.MORPH_OPEN, kernel)  # 开操作去除小噪点
        color_mask = cv.morphologyEx(color_mask, cv.MORPH_CLOSE, kernel) # 闭操作填充空洞
        
        # 在HSV掩码上检测轮廓
        # Detect contours directly on HSV mask
        find_contours = cv.findContours(color_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if len(find_contours) == 3: contours = find_contours[1]
        else: contours = find_contours[0]
        
        # 只处理面积最大的轮廓
        # Only process the largest contour
        if contours:
            # 找到面积最大的轮廓
            # Find the contour with the largest area
            cnt = max(contours, key=cv.contourArea)
            
            # Calculate the moment of a polygon
            # 计算多边形的矩
            mm = cv.moments(cnt)
            if mm['m00'] == 0: return None
            cx = mm['m10'] / mm['m00']
            cy = mm['m01'] / mm['m00']
            # Get the center of the polygon
            # 获取多边形的中心
            (x, y) = (int(cx), int(cy))
            # Calculate the area of the contour
            # 计算轮廓的⾯积
            area = cv.contourArea(cnt)
            
            # 面积过滤：只处理面积大于1000的轮廓
            # Area filtering: only process contours with area > 1000
            if area > 1000:
                # Calculate the smallest rectangular area
                # 计算最小矩形区域
                rect = cv.minAreaRect(cnt)
                # get box vertices
                # 获取盒⼦顶点
                box = cv.boxPoints(rect)
                # Convert to long type
                # 转成long类型
                box = np.int0(box)
                
                # drawing center
                # 绘制中⼼
                cv.circle(self.image, (x, y), 5, (0, 0, 255), -1)
                
                dis = 27.03 * math.pow(area, -0.51)
                # 拼接文字：距离（米）和颜色 Text content: distance (m) and color
                text_content = f"dis:{dis:.2f}m {hsv_name}"
                # draw the smallest rectangle
                # 绘制最小矩形
                cv.drawContours(self.image, [box], 0, (255, 0, 0), 2)
                cv.putText(self.image, text_content, (int(box[1][0] - 15), int(box[1][1]) - 15),
                           cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                return area
        
        return None

    def target_run(self, img, color_hsv, target_color=None):
        self.image = cv.resize(img, (640, 480), )
        # Only process the specified target color, if provided
        # 只处理指定的目标颜色（如果提供了的话）
        msg = {}
        if target_color and target_color in color_hsv:
            # Only process the specified color
            # 只处理指定的颜色
            area = self.get_area(target_color, color_hsv[target_color])
            if area != None:
                msg[target_color] = area
        # else:
        #     # If no target color specified, process all colors (original behavior)
        #     for key, value in color_hsv.items():
        #         area = self.get_area(key, value)
        #         if area != None:
        #             msg[key] = area
        return self.image, msg