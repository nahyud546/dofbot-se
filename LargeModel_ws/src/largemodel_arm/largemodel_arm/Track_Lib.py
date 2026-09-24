#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2 as cv

class Tracker(object):
    '''
    追踪者模块,用于追踪指定目标
    Tracker module, used to track the specified target
    '''
    def __init__(self, tracker_type="BOOSTING"):
        '''
        初始化追踪器种类
        Initialize the type of tracker
        '''
        # 获得opencv版本
	# Get the opencv version
        (major_ver, minor_ver, subminor_ver) = (cv.__version__).split('.')
        self.tracker_type = tracker_type
        self.isWorking = False
        self.init_frame_size = None
        self.init_frame_channels = None
        # 构造追踪器
	# Construct tracker
        # print(tracker_type)
        if int(major_ver) < 3: self.tracker = cv.Tracker_create(tracker_type)
        else:
            if tracker_type == 'BOOSTING': self.tracker = cv.TrackerBoosting_create()
            if tracker_type == 'MIL': self.tracker = cv.TrackerMIL_create()
            if tracker_type == 'KCF': self.tracker = cv.TrackerKCF_create()
            if tracker_type == 'TLD': self.tracker = cv.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW': self.tracker = cv.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN': self.tracker = cv.TrackerGOTURN_create()
            if tracker_type == 'MOSSE': self.tracker = cv.TrackerMOSSE_create()
            if tracker_type == "CSRT": self.tracker = cv.TrackerCSRT_create()
        

    def initWorking(self, frame, box):
        '''
        Tracker work initialization 追踪器工作初始化
        frame:初始化追踪画面
        box:追踪的区域

        '''
        
        if not self.tracker: raise Exception("追踪器未初始化Tracker is not initialized")
        status = self.tracker.init(frame, box)
        #if not status: raise Exception("追踪器工作初始化失败Failed to initialize tracker job")
        self.coord = box
        self.isWorking = True
        self.init_frame_size = (frame.shape[0], frame.shape[1])  # (h, w)
        self.init_frame_channels = frame.shape[2] if len(frame.shape) == 3 else 1

    def track(self, frame):
        #print("self.init_frame_channels: ",self.init_frame_channels)
        #print("self.init_frame_size: ",self.init_frame_size)
        frame_channels = frame.shape[2] if len(frame.shape) == 3 else 1
        #print("frame_channels: ",frame_channels)
        frame_size = (frame.shape[0], frame.shape[1])
        #print("frame_size: ",frame_size)        
        if self.isWorking:
            if self.init_frame_channels == 3 and len(frame.shape) == 2:
                # 灰度图转彩色
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif self.init_frame_channels == 1 and len(frame.shape) == 3:
                # 彩色图转灰度
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            current_h, current_w = frame.shape[0], frame.shape[1]
            if (current_h, current_w) != self.init_frame_size:
                # 缩放帧到初始尺寸（保持比例，避免变形）
                frame = cv2.resize(frame, (self.init_frame_size[1], self.init_frame_size[0]))
            status, self.coord = self.tracker.update(frame)
            if status:
                p1 = (int(self.coord[0]), int(self.coord[1]))
                p2 = (int(self.coord[0] + self.coord[2]), int(self.coord[1] + self.coord[3]))
                cv.rectangle(frame, p1, p2, (255, 0, 0), 2, 1)
                return frame, p1, p2
            else:
                # 跟踪失败
		# Tracking failed
                cv.putText(frame, "Tracking failure detected", (100, 80), cv.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
                return frame, (0, 0), (0, 0)
        else: return frame, (0, 0), (0, 0)