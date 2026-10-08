Đừng nhảy ngay vào ROS/MoveIt.
Hãy làm một mini laboratory 3D trước:
                 PROJECT
                    │
       ┌────────────┴────────────┐
       │                         │
   Camera A                  Camera B
       │                         │
       └──────────┬──────────────┘
                  ↓
             Stereo 3D
                  ↓
            Point cloud
                  ↓
             WORLD FRAME
                  ↓
             Robot frame
                  ↓
                ROS TF
                  ↓
              MoveIt 2

Bài 1
Một camera.
Học:
K
distortion
projection
back projection

Bài 2
Hai camera cố định.
Học:
stereo calibration
R
T
epipolar geometry
triangulation

Bài 3
Đặt một cube trên bàn.
Hai camera → tính:
X Y Z

và visualize bằng Open3D.
Bài 4
Di chuyển cube.
Camera vẫn đứng yên.
→ Object tracking:
P(t0)
P(t1)
P(t2)
...

Bài 5
Di chuyển một camera.
Object đứng yên.
→ Camera pose tracking.
Đây là lúc bạn bắt đầu hiểu:
T_world_camera(t)

Bài 6
Thêm robot.
world
 ├── cam1
 ├── cam2
 ├── object
 └── robot_base

Bài 7
Camera → object → robot.
pixel
 ↓
3D
 ↓
world
 ↓
base_link
 ↓
IK
 ↓
grasp

Bài 8
Cuối cùng mới:
moving camera
+
moving robot
+
moving object

Lúc đó bạn đã bước sang dynamic 3D perception.