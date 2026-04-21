# navigation_Final-project — 架構說明

## 專案概述

以 ROS2 為核心的自主導航機器人系統，透過 Docker Compose 編排多個感測器與演算法服務。整個系統分為三個子模組，各自獨立運行於不同的 Docker 容器中，並透過 ROS2 DDS 網路互相溝通。

---

## 目錄結構

```
pros/
├── pros_app/                  # 主機端控制層（啟動/管理所有 Docker 服務）
│   ├── control.py             # 互動式選單入口（Python，推薦使用）
│   ├── control.sh             # 互動式選單入口（Bash）
│   ├── utils.sh               # docker compose up/down 共用函式
│   ├── slam.sh                # 啟動 RPLidar + slam_toolbox
│   ├── slam_ydlidar.sh        # 啟動 YDLidar + slam_toolbox
│   ├── slam_oradarlidar.sh    # 啟動 OradarLidar + slam_toolbox
│   ├── slam_unity.sh          # Unity 模擬 SLAM
│   ├── localization.sh        # 啟動 RPLidar + AMCL + Nav2
│   ├── localization_ydlidar.sh
│   ├── localization_oradarlidar.sh
│   ├── localization_unity.sh
│   ├── store_map.sh           # 儲存 SLAM 所建地圖
│   ├── camera_astra.sh        # Astra 深度相機驅動
│   ├── camera_dabai.sh        # DaBai 相機驅動
│   ├── camera_gemini.sh       # Gemini 相機驅動
│   ├── imu.sh                 # IMU 感測器
│   ├── gps.sh                 # GPS 模組
│   ├── rosbridge_server.sh    # WebSocket bridge（port 9090）
│   ├── ros_topic_bridge.sh    # ROS2 topic 橋接
│   └── docker/
│       ├── Dockerfile         # pros_app 自定義映像
│       └── compose/           # 所有 docker-compose yml 檔案
│           ├── docker-compose_rplidar.yml
│           ├── docker-compose_ydlidar.yml
│           ├── docker-compose_oradarlidar.yml
│           ├── docker-compose_slam.yml
│           ├── docker-compose_localization.yml
│           ├── docker-compose_navigation.yml
│           ├── docker-compose_slam_unity.yml
│           ├── docker-compose_localization_unity.yml
│           ├── docker-compose_navigation_unity.yml
│           ├── docker-compose_camera_astra.yml
│           ├── docker-compose_camera_dabai.yml
│           ├── docker-compose_camera_gemini.yml
│           ├── docker-compose_imu.yml
│           ├── docker-compose_gps.yml
│           ├── docker-compose_rosbridge_server.yml
│           ├── docker-compose_store_map.yml
│           └── docker-compose_ros_topic_bridge.yml
│
├── pros_car/                  # 機器人控制層（在 Docker 容器內執行 ROS2）
│   ├── car_control.sh         # 啟動容器的 Bash 腳本
│   ├── activate.py            # 啟動容器的 Python 腳本（功能更完整）
│   ├── Dockerfile
│   ├── .env                   # 環境變數（ROS_DOMAIN_ID 等）
│   ├── launch/
│   │   ├── activate.sh
│   │   └── car_control_launch.launch.py
│   └── src/                   # ROS2 workspace
│       ├── car_control_pkg/   # 導航控制 package
│       ├── pros_car_py/       # 核心 Python 函式庫 package
│       ├── action_interface/  # 自定義 Action 訊息
│       ├── custome_interfaces/# 自定義 Message/Service
│       ├── keyboard_mode_interface_pkg/  # 鍵盤輸入介面
│       ├── robot_description/ # URDF 機器人模型
│       ├── ros_communication_pkg/
│       └── arm_control_pkg/   # 機械臂控制
│
└── ros2_yolo_integration/     # 視覺偵測層（YOLO + 深度相機）
    ├── yolo_activate.sh       # 啟動容器入口
    ├── Dockerfile
    ├── .env
    └── src/
        ├── yolo_pkg/          # 主要 YOLO 偵測 package
        ├── arucode_pkg/       # ArUco 標記辨識
        ├── depth_test_pkg/    # 深度相機測試
        └── yolo_example_pkg/  # 使用範例
```

---

## 子模組詳解

### 1. `pros/pros_app` — 主機端服務管理層

**用途**：在主機上啟動/停止所有 Docker Compose 服務，作為整個系統的入口點。

**啟動方式**：
```bash
cd pros/pros_app
python3 control.py        # 互動選單（推薦）
bash control.sh           # 或用 bash 版本
```

**運作原理**：
- `utils.sh` 的 `main()` 函式依序執行 `docker compose -f <yml> up -d`，並在 Ctrl+C 時自動執行 `docker compose down`。
- 每個 `.sh` 腳本（如 `slam.sh`）只是 source `utils.sh` 後呼叫 `main` 並傳入對應的 yml 路徑。

**Docker 網路**：所有服務都掛在 `compose_my_bridge_network`，容器間透過 ROS2 DDS 直接通訊。

---

### 2. `pros/pros_car` — 機器人控制層

**Docker 映像**：`ghcr.io/screamlab/pros_car_docker_image:latest`

**啟動方式**：
```bash
cd pros/pros_car
python3 activate.py       # 自動偵測架構（arm64/x86_64/macOS）與 GPU
bash car_control.sh       # 或用 Bash 版本
```

`activate.py` 會自動：
- 偵測系統架構（aarch64 / x86_64 / Darwin）
- 偵測 NVIDIA GPU（Tegra / NVIDIA runtime）
- 掛載 `./src` 與 `./launch` 進容器
- 掛載硬體裝置（`/dev/usb_front_wheel`, `/dev/usb_rear_wheel`, `/dev/usb_robot_arm`）

#### ROS2 Package：`car_control_pkg`

| 檔案 | 功能 |
|------|------|
| `car_action_server.py` | ROS2 Action Server，接收高階動作指令 |
| `car_nav_controller.py` | 整合 Nav2，執行點到點自動導航 |
| `car_manual.py` | 鍵盤手動控制節點 |
| `car_control_common.py` | 共用控制邏輯 |
| `nav2_utils.py` | Nav2 SimpleNavigator 封裝 |
| `action_config.py` | Action 參數設定 |

#### ROS2 Package：`pros_car_py`

| 檔案 | 功能 |
|------|------|
| `ros_communicator.py` | 所有 ROS2 Publisher/Subscriber 管理 |
| `ros_communicator_config.py` | 四輪速度映射表（ACTION_MAPPINGS） |
| `car_controller.py` | 手動/自動/目標導航控制器 |
| `car_models.py` | `DeviceDataTypeEnum`、`CarCControl` 等資料模型 |
| `nav_processing.py` | Nav2 路徑追蹤與動作轉換 |
| `nav2_utils.py` | Nav2 工具函式 |
| `arm_controller.py` | 機械臂 3D 控制（IK） |
| `arm_controller_2D.py` | 機械臂 2D 控制 |
| `crane_controller.py` | 吊臂控制 |
| `ik_solver.py` | 逆運動學求解器 |
| `data_processor.py` | 感測器資料前處理 |
| `mode_manager.py` | 模式切換管理器 |
| `carC_serial_reader/writer.py` | Arduino 序列通訊 |

**速度設定**（`ros_communicator_config.py`）：
```python
speed_ratio = 50
vel = 6.0 * speed_ratio   # = 300（前進速度）
# 輪序：前左、前右、後左、後右
ACTION_MAPPINGS = {
    "FORWARD":  [300, 300, 300, 300],
    "BACKWARD": [-300, -300, -300, -300],
    "COUNTERCLOCKWISE_ROTATION": [-300, 300, -300, 300],
    "CLOCKWISE_ROTATION":        [300, -300, 300, -300],
    "STOP":     [0, 0, 0, 0],
    # ...（另有 SLOW/MEDIAN 變體與橫移）
}
```

**導航模式**（`car_controller.py`）：
- `manual_control(key)` — 鍵盤直接控制（w/a/s/d/e/r/z）
- `auto_control(mode="manual_auto_nav")` — 由 Nav2 規劃路徑後自動跟隨
- `auto_control(mode="target_auto_nav")` — 自動巡迴預設座標列表
- `auto_control(mode="custom_nav")` — 客製化視覺導航

---

### 3. `pros/ros2_yolo_integration` — 視覺偵測層

**啟動方式**：
```bash
cd pros/ros2_yolo_integration
bash yolo_activate.sh
```

#### ROS2 Package：`yolo_pkg`

| 檔案 | 功能 |
|------|------|
| `main.py` | 節點入口 |
| `image_processor.py` | 影像處理管線 |
| `yolo_bounding_box.py` | YOLO 偵測框計算 |
| `yolo_depth_extractor.py` | 從深度影像提取 3D 座標 |
| `camera_geometry.py` | 相機投影/幾何變換 |
| `camera_parameters.py` | 相機內部參數 |
| `boundingbox_visaulizer.py` | 偵測框視覺化 |
| `ros_communicator.py` | ROS2 影像訂閱與結果發布 |
| `load_params.py` | 載入 YAML 參數 |
| `models/` | YOLO 權重檔 |

其他 packages：
- `arucode_pkg` — ArUco marker 辨識（用於精確定位）
- `depth_test_pkg` — 深度相機測試工具
- `yolo_example_pkg` — 整合使用範例

---

## 完整系統架構圖

```
主機 (Host Machine)
│
├── pros_app/control.py  ← 使用者操作入口
│   │
│   └── docker compose (compose_my_bridge_network)
│       ├── rplidar / ydlidar / oradarlidar   ← /scan topic
│       ├── slam_toolbox                       ← /map topic
│       ├── AMCL (localization)               ← /amcl_pose topic
│       ├── Nav2 (navigation stack)           ← /cmd_vel topic
│       ├── camera driver (astra/dabai/gemini)← /image_raw, /depth topic
│       ├── IMU                               ← /imu topic
│       ├── GPS                               ← /gps topic
│       └── rosbridge_server (ws://9090)      ← 外部 App 連入
│
├── pros_car/  (ghcr.io/screamlab/pros_car_docker_image)
│   ├── car_control_pkg 節點
│   │   ├── 訂閱 /cmd_vel → 轉為輪速指令
│   │   └── 訂閱 Nav2 Action → 自動導航
│   ├── pros_car_py 節點
│   │   ├── 序列通訊 → /dev/usb_front_wheel, /dev/usb_rear_wheel
│   │   └── 序列通訊 → /dev/usb_robot_arm
│   └── arm_control_pkg → 機械臂 IK 控制
│
└── ros2_yolo_integration/
    └── yolo_pkg 節點
        ├── 訂閱 /image_raw, /depth
        ├── YOLO 推論 → /yolo_detections
        └── ArUco 偵測 → /aruco_markers
```

---

## 標準操作流程

### 第一步：建圖（SLAM）
```bash
cd pros/pros_app
python3 control.py
# 選 1 (slam.sh) → 啟動 LiDAR + slam_toolbox
# 手動遙控機器人繞行環境
# 選 4 (store_map.sh) → 儲存地圖
```

### 第二步：自主導航
```bash
# 選 5 (localization.sh) → 啟動 AMCL + Nav2
# 在 RViz 或 App 設定目標點
```

### 第三步：啟動機器人控制節點
```bash
cd pros/pros_car
python3 activate.py   # 進入容器
# 容器內：
ros2 launch launch/car_control_launch.launch.py
```

### 第四步（選用）：啟動 YOLO 視覺
```bash
cd pros/ros2_yolo_integration
bash yolo_activate.sh
```

---

## 關鍵技術

| 技術 | 用途 |
|------|------|
| ROS2 (Humble) | 節點通訊框架 |
| Nav2 | 自主導航堆疊（AMCL + costmap + planner） |
| slam_toolbox | 線上/離線 SLAM 建圖 |
| YOLOv8 | 即時物件偵測 |
| ArUco | 精確標記定位 |
| Docker Compose | 服務編排與隔離 |
| Mecanum Drive | 全向移動底盤（4輪） |
| rosbridge | WebSocket → ROS2 橋接（供 Unity/App 使用） |
| Unity | 模擬環境支援 |

---

## 硬體裝置對應

| 裝置節點 | 對應硬體 |
|----------|----------|
| `/dev/usb_front_wheel` | 前輪馬達控制器 (Arduino) |
| `/dev/usb_rear_wheel` | 後輪馬達控制器 (Arduino) |
| `/dev/usb_robot_arm` | 機械臂控制器 |
| LiDAR USB | RPLidar / YDLidar / OradarLidar |
| USB Camera | Astra / DaBai / Gemini 深度相機 |

---

## Docker 映像說明

| 映像 | 用途 |
|------|------|
| `ghcr.io/screamlab/pros_car_docker_image:latest` | pros_car ROS2 環境（支援 arm64/x86_64，可選 GPU） |
| pros_app 自建映像（`docker/Dockerfile`） | pros_app 的 ROS2 服務容器 |
| ros2_yolo_integration 自建映像（`Dockerfile`） | YOLO 推論環境 |

所有容器共用 Docker bridge network `compose_my_bridge_network`，ROS_DOMAIN_ID 透過 `.env` 統一設定。
