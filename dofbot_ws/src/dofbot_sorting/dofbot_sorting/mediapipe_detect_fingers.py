import cv2
import mediapipe as mp
import time

# 定义手指名称与对应的关键点ID
FINGER_LANDMARKS = {
    '拇指': [1, 2, 3, 4],
    '食指': [5, 6, 7, 8],
    '中指': [9, 10, 11, 12],
    '无名指': [13, 14, 15, 16],
    '小指': [17, 18, 19, 20]
}


class FingersDetector:
    
    def __init__(self, mode=False, maxHands=2, detectorCon=0.5, trackCon=0.5):
        self.mpHand = mp.solutions.hands
        self.draw = False
        self.mpDraw = mp.solutions.drawing_utils
        self.hands = self.mpHand.Hands(
            static_image_mode=mode,
            max_num_hands=maxHands,
            min_detection_confidence=detectorCon,
            min_tracking_confidence=trackCon
        )
    
    
    def get_hand_label(self,handedness):
        """判断是左手还是右手"""
        label = handedness.classification[0].label
        return '左手' if label == 'Left' else '右手'
    
    def is_finger_extended(self,hand_landmarks, finger_name, hand_label):
        """判断单根手指是否伸直"""
        landmarks = hand_landmarks.landmark
        finger_ids = FINGER_LANDMARKS[finger_name]
        
        if finger_name == '拇指':
            # 拇指伸直判断逻辑：指尖(4)与食指根部(5)的水平距离
            thumb_tip = landmarks[4]
            index_mcp = landmarks[5]  # 食指掌指关节
            
            # 区分左右手，调整拇指判断的水平方向
            if hand_label == '左手':
                # 左手拇指伸直：指尖x坐标 < 食指根部x坐标
                return thumb_tip.x < index_mcp.x - 0.01
            else:
                # 右手拇指伸直：指尖x坐标 > 食指根部x坐标
                return thumb_tip.x > index_mcp.x + 0.01
        else:
            # 其余手指伸直判断：指尖点y坐标 < 中关节点y坐标（值越小越靠上）
            tip = landmarks[finger_ids[-1]]       # 指尖
            pip = landmarks[finger_ids[-3]]      # 中关节（近节指间关节）
            return tip.y < pip.y - 0.03  # 增加阈值避免误判
    
    def count_extended_fingers(self,hand_landmarks, hand_label):
        """统计单只手中伸直的手指数量"""
        extended_count = 0
        extended_fingers = []
        
        for finger_name in FINGER_LANDMARKS.keys():
            if self.is_finger_extended(hand_landmarks, finger_name, hand_label):
                extended_count += 1
                extended_fingers.append(finger_name)
        
        return extended_count, extended_fingers

    def detect_extended_fingers(self,frame):
        h, w, _ = frame.shape
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)
        frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        total_extended = 0  # 所有手的伸直手指总数
        hands_info = []     # 存储每只手的信息
        # 检测到手部时
        if results.multi_hand_landmarks and results.multi_handedness:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                # 判断左右手
                hand_label = self.get_hand_label(handedness)
                # 统计伸直手指数量
                count, fingers = self.count_extended_fingers(hand_landmarks, hand_label)
                total_extended += count
                hands_info.append({
                    'hand': hand_label,
                    'count': count,
                    'fingers': fingers
                })
        print("total_extended: ",total_extended)
        return total_extended                