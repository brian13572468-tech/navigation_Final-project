# pros_car 程式架構筆記

---

## 一、啟動指令對應關係

| 啟動指令 | 對應檔案 |
|----------|----------|
| `pros_app` 選 `slam_unity.sh` | `pros_app/slam_unity.sh` → `utils.sh` → docker compose 啟動 LiDAR + SLAM + rosbridge |
| `./car_control.sh` 進容器 | `pros_car/car_control.sh` → 進入 Docker 容器 |
| `ros2 run pros_car_py robot_control` | `pros_car_py/main2.py:main()` |

---

## 二、`robot_control` 的架構（main2.py）

### 整體結構：1 個 Process，2 個 Thread，1 個 ROS2 Node

```
Process: ros2 run pros_car_py robot_control
│
├── Thread 1 (main thread)    → urwid TUI 事件迴圈（ModeApp.main()）
└── Thread 2 (daemon thread)  → rclpy.spin(ros_communicator)
```

### main2.py 的初始化順序

```python
# 1. 啟動唯一的 ROS2 Node
ros_communicator = RosCommunicator()       # 負責所有 ROS2 收發
thread = Thread(target=rclpy.spin, ...)    # 跑在背景

# 2. 初始化所有控制器（全部接收 ros_communicator）
data_processor    = DataProcessor(ros_communicator)
nav2_processing   = Nav2Processing(ros_communicator, data_processor)
ik_solver         = PybulletRobotController(end_eff_index=5)
car_controller    = CarController(ros_communicator, nav2_processing)
arm_controller    = ArmController(ros_communicator, data_processor)
crane_controller  = CraneController(ros_communicator, data_processor, ik_solver, num_joints=7)
custom_control    = CustomControl(car_controller, arm_controller)

# 3. 啟動 TUI（阻塞主執行緒）
app = ModeApp(car_controller, arm_controller, custom_control, crane_controller)
app.main()
```

### 物件關係圖

```
main2.py（工廠）
│
├─ 建立 RosCommunicator（唯一 ROS2 Node）
│   ├─ 訂閱：/amcl_pose, /goal_pose, /scan, /received_global_plan,
│   │         /cmd_vel, /camera/image/compressed, /camera/x_multi_depth_values,
│   │         /yolo/detection/position, /yolo/detection/offset,
│   │         /yolo/detection/status, /yolo/target_info, /yolo/target_marker,
│   │         /imu/data, /mediapipe_data, /clicked_point
│   └─ 發布：/car_C_rear_wheel, /car_C_front_wheel, /robot_arm,
│             /goal_pose, /plan, crane_state, /arm_visual_lines
│
├─ 建立 CarController  ─────┐
├─ 建立 ArmController  ─────┤
├─ 建立 CraneController ────┤  全部接受 ros_communicator 作為橋接
├─ 建立 Nav2Processing ─────┤
│                            │
└─ 建立 ModeApp（控制台）   ◄─┘
    └─ 把所有 controller 存為自己的屬性
```

---

## 三、TUI 介面架構

### 畫面控制核心（urwid）

urwid 的 MainLoop 只有兩個控制點：

```python
self.app.loop.widget           # 決定「畫面顯示什麼」
self.app.loop.unhandled_input  # 決定「按鍵由誰處理」
```

每次切換模式，就是替換這兩個東西。

### 選單結構（mode_app.py）

```
Main Menu（ModeApp）
├─ Control Vehicle      → VehicleMode
├─ Manual Arm Control   → ArmMode
├─ Manual Crane Control → CraneMode
├─ Auto Navigation      → AutoNavMode
├─ Automatic Arm Mode   → AutoArmMode
└─ Exit
```

---

## 四、`app` 物件的傳遞方式（重要觀念）

### 用「盒子」比喻

```
main2.py（工廠）
│
├─ 製造 遙控器（CarController）
│
└─ 製造 控制台（ModeApp），並把遙控器放進去
       │
       └─ 使用者點 VehicleMode
              │
              └─ 控制台把「自己整個」交給 VehicleMode
                     │
                     └─ 使用者按鍵
                            │
                            └─ VehicleMode 從控制台取出遙控器使用
```

### 程式碼對應

```python
# main2.py
car_controller = CarController(...)          # 遙控器
app = ModeApp(car_controller, ...)           # 控制台，內含遙控器

# mode_app.py
class ModeApp:
    def __init__(self, car_controller, ...):
        self.car_controller = car_controller  # 遙控器存進控制台

    def switch_mode(self, mode_name):
        self.current_mode = VehicleMode(self) # 把「自己（控制台）」傳給 Mode

# base_mode.py
class BaseMode:
    def __init__(self, app):
        self.app = app                        # app = 控制台（ModeApp）

# mode_manager.py
class VehicleMode(BaseMode):
    def handle_input(self, key):
        self.app.car_controller.manual_control(key)
        # self.app             = 控制台（ModeApp）
        # self.app.car_controller = 控制台裡的遙控器（CarController）
```

---

## 五、VehicleMode 按鍵流程

### 涉及檔案

| 順序 | 檔案 | 做什麼 |
|------|------|--------|
| 1 | `mode_app.py` | `switch_mode()` 建立 VehicleMode 並呼叫 `enter()` |
| 2 | `mode_manager.py` | `VehicleMode.enter()` 設定畫面；`handle_input()` 接收按鍵 |
| 3 | `base_mode.py` | 提供共用畫面工具 |
| 4 | `car_controller.py` | `manual_control()` 把按鍵對應成動作字串 |
| 5 | `ros_communicator_config.py` | `ACTION_MAPPINGS` 定義動作對應的四輪速度值 |
| 6 | `ros_communicator.py` | `publish_car_control()` 發布到 ROS2 topics |

### 呼叫鏈

```
使用者按 "w"
  │
  ▼ urwid 主執行緒
VehicleMode.handle_input("w")          ← mode_manager.py
  │
  ▼
CarController.manual_control("w")      ← car_controller.py
  │ key="w" → action="FORWARD"
  ▼
CarController.update_action("FORWARD")
  │
  ▼
RosCommunicator.publish_car_control("FORWARD")   ← ros_communicator.py
  │  查 ACTION_MAPPINGS["FORWARD"] = [300,300,300,300]  ← ros_communicator_config.py
  │
  ├─ 發布 /car_C_rear_wheel  ← Float32MultiArray([300, 300])
  └─ 發布 /car_C_front_wheel ← Float32MultiArray([300, 300])
          │
          ▼
    carC_writer node → Serial → Arduino → 四輪馬達
```

### 按鍵對應表

| 按鍵 | 動作 |
|------|------|
| `w` | FORWARD（前進） |
| `s` | BACKWARD（後退） |
| `a` | LEFT_FRONT（左前） |
| `d` | RIGHT_FRONT（右前） |
| `e` | COUNTERCLOCKWISE_ROTATION（逆時針自轉） |
| `r` | CLOCKWISE_ROTATION（順時針自轉） |
| `z` | STOP |
| `q` | STOP + 回主選單 |

---

## 六、Automatic Arm Mode 流程

### 涉及檔案

| 順序 | 檔案 | 做什麼 |
|------|------|--------|
| 1 | `mode_manager.py` | `AutoArmMode` 設定畫面、接收按鍵、轉給 arm_controller |
| 2 | `base_mode.py` | `horizontal_select`、`show_submode_screen` 畫面工具 |
| 3 | `arm_controller_2D.py` | 核心邏輯：IK 計算、抓取序列、平滑移動、發布角度 |
| 4 | `ros_communicator.py` | 讀取 YOLO 目標、發布 `/robot_arm` topic |

### 呼叫鏈

```
使用者選 Automatic Arm Mode
  │
  ▼ mode_manager.py
AutoArmMode.enter()
  → 畫面顯示 horizontal_select ["auto_arm_human"]
  │
使用者按 Enter 選 auto_arm_human
  │
  ▼ mode_manager.py
handle_submode_select("auto_arm_human")
  → 畫面顯示操作說明，等待按鍵
  │
使用者按 "g"
  │
  ▼ arm_controller_2D.py
ArmController.auto_control(key="g")
  → 讀取 ros_communicator.latest_yolo_marker（YOLO 偵測到的目標位置）
  → TF2 座標轉換（map 座標系 → 手臂基準座標系）
  → 開新背景執行緒：_execute_grab_sequence(x, z)
        │
        ├─ [1/4] 打開夾爪  → _smooth_move_to → Joint 2 到 max_angle
        ├─ [2/4] 移到目標  → _calculate_2d_ik(x,z) → _smooth_move_to
        ├─ [3/4] 夾住目標  → _smooth_move_to → Joint 2 到 min_angle
        └─ [4/4] 回初始    → _smooth_move_to → Joint 0, 1 回 init
              │（每個步驟都呼叫）
              ▼
        _clamp_and_publish()
          → 角度轉 radian
          → ros_communicator.publish_robot_arm_angle([radians...])
          → 發布到 /robot_arm topic
          → arm_writer node → Serial → ESP32 → 馬達
```

### 按鍵對應表（auto_arm_human 模式下）

| 按鍵 | 動作 |
|------|------|
| `g` | 開始自動抓取（需要 YOLO 先偵測到目標） |
| `b` | 重置手臂回初始位置 |
| `q` | 離開，回上一層選單 |

### 2D IK 計算說明（_calculate_2d_ik）

機械臂有 3 個關節：

| 關節 | 說明 | 臂長 |
|------|------|------|
| Joint 0 | 肩膀（Shoulder） | 0.08089 m |
| Joint 1 | 手肘（Elbow） | 0.11 m |
| Joint 2 | 夾爪（Gripper） | 無臂長 |

IK 計算流程：
1. 用餘弦定理算 theta2（手肘角度）
2. 用 atan2 + 修正算 theta1（肩膀角度）
3. 加上各關節的 `offset` 補償值，轉成馬達實際角度
4. 用 `_normalize_angle` 確保在關節限制範圍內

---

## 七、各 Mode 的子選單結構

| Mode | 子選單內容 | 控制對象 |
|------|-----------|----------|
| VehicleMode | 無子選單，直接接受 w/a/s/d/e/r/z | CarController |
| ArmMode | 選關節 0~4 → i/k 控制角度 | ArmController.manual_control(index, key) |
| CraneMode | 選模式 0~6, 99 → 鍵盤控制 | CraneController.manual_control(index, key) |
| AutoNavMode | 選 manual_auto_nav / target_auto_nav / custom_nav | CarController.auto_control(mode, key) |
| AutoArmMode | 選 auto_arm_human → g 抓取 | ArmController.auto_control(mode, key) |
