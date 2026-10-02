# Hikrobot Camera ROS 2 Driver

基于 Hikrobot MVS SDK 和 ROS 2 Humble 实现的工业相机 ROS 2 驱动节点。

本项目用于将 Hikrobot 工业相机采集的图像转换为标准 ROS 2 `sensor_msgs/msg/Image` 消息，并发布到 `/image_raw` 话题。

---

## 1. 项目功能

本项目实现以下主要功能：

- Hikrobot MVS SDK 相机发现
- USB / GigE 相机设备枚举
- 通过相机序列号选择设备
- 通过 GigE IP 地址选择设备
- 相机打开与关闭
- 图像连续采集
- 发布标准 ROS 2 `sensor_msgs/msg/Image`
- Mono8 图像发布
- 图像尺寸自动读取
- 可配置曝光时间
- 可配置增益
- 可配置采集帧率
- 可配置像素格式
- 参数运行时动态修改
- 参数合法性检查
- ROS 2 Launch 启动
- YAML 参数配置
- 相机连接状态监控
- 相机断开后的重新枚举与重连机制
- 设备不存在、标识冲突或被占用时的明确错误反馈
- 连续取流失败后的自动重连

---

## 2. 开发环境

当前测试环境：

- Ubuntu 22.04
- ROS 2 Humble
- Hikrobot MVS SDK
- C++17
- CMake
- colcon

MVS SDK 默认安装路径：

```text
/opt/MVS
```

MVS 动态库：

```text
/opt/MVS/lib/64/libMvCameraControl.so
```

---

## 3. 项目结构

```text
assignment3-ROS2/
├── README.md
├── src/
│   └── hikrobot_camera/
│       ├── CMakeLists.txt
│       ├── package.xml
│       │
│       ├── include/
│       │   └── hikrobot_camera/
│       │       └── camera_node.hpp
│       │
│       ├── src/
│       │   ├── camera_node.cpp
│       │   └── main.cpp
│       │
│       ├── launch/
│       │   └── camera.launch.py
│       │
│       └── config/
│           └── camera.yaml
```

---

## 4. 编译

进入 ROS 2 工作空间：

```bash
cd ~/assignment3-ROS2
```

先加载 ROS 2 Humble 环境，并安装 `package.xml` 中的依赖：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

> MVS SDK 不在 apt/rosdep 依赖中，需要单独安装到 `/opt/MVS`。如果安装在
> 其他路径，可用 `-DMVS_ROOT=/path/to/MVS` 覆盖。

编译：

```bash
colcon build --packages-select hikrobot_camera
```

编译成功后：

```bash
source install/setup.zsh
```

如果使用 Bash：

```bash
source install/setup.bash
```

---

## 5. 配置相机

配置文件：

```text
src/hikrobot_camera/config/camera.yaml
```

当前 USB 相机配置：

```yaml
/hikrobot_camera:
  ros__parameters:

    use_sim_time: false

    camera_ip: ""

    serial_number: "00F78118392"

    image_topic: "/image_raw"

    exposure_time: 10000.0

    gain: 0.0

    frame_rate: 30.0

    pixel_format: "Mono8"
```

---

## 6. 相机选择

### 6.1 通过序列号选择

推荐 USB 相机使用序列号：

```yaml
serial_number: "00F78118392"
camera_ip: ""
```

程序启动时会枚举 MVS 设备，并寻找指定序列号的相机。

---

### 6.2 通过 IP 地址选择

GigE 相机可以配置：

```yaml
camera_ip: "192.168.1.100"
serial_number: ""
```

程序会寻找 IP 地址匹配的 GigE 相机。

---

### 6.3 两个参数同时指定

如果同时指定：

```yaml
camera_ip: "192.168.1.100"
serial_number: "00F78118392"
```

程序要求设备同时满足 IP 和序列号条件。

如果未配置任何标识且发现多台相机，节点会报告标识冲突并拒绝随机选择；
如果设备不存在、已被其他程序占用或权限不足，节点会输出明确错误信息。

---

## 7. 启动节点

推荐使用 Launch：

```bash
ros2 launch hikrobot_camera camera.launch.py
```

正常情况下可以看到类似：

```text
Starting Hikrobot MVS camera node...
Found 1 camera(s).
Camera 0: serial=00F78118392
Camera device opened.
Hikrobot camera connected and started grabbing.
Hikrobot camera connected successfully.
```

---

## 8. 查看 ROS 2 节点

另开一个终端：

```bash
source ~/assignment3-ROS2/install/setup.zsh
```

查看节点：

```bash
ros2 node list
```

正常情况下：

```text
/hikrobot_camera
```

---

## 9. 查看图像话题

执行：

```bash
ros2 topic list
```

应该能够看到：

```text
/image_raw
```

查看话题信息：

```bash
ros2 topic info /image_raw
```

查看消息类型：

```bash
ros2 topic type /image_raw
```

应该得到：

```text
sensor_msgs/msg/Image
```

---

## 10. 查看图像帧率

执行：

```bash
ros2 topic hz /image_raw
```

默认配置为：

```yaml
frame_rate: 30.0
```

因此正常情况下 ROS 2 图像话题帧率应该接近：

```text
30 Hz
```

实际帧率会受到相机、USB、系统负载和 ROS 2 调度等因素影响。

---

## 11. 查看图像

安装并启动 `rqt_image_view` 后：

```bash
rqt_image_view
```

选择：

```text
/image_raw
```

也可以直接使用：

```bash
/opt/ros/humble/lib/rqt_image_view/rqt_image_view
```

当前相机测试图像：

```text
Width:  1440
Height: 1080
Encoding: mono8
```

---

## 12. ROS 2 参数

当前支持以下参数：

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---:|---|
| `camera_ip` | string | `""` | GigE 相机 IP |
| `serial_number` | string | `00F78118392` | 相机序列号 |
| `image_topic` | string | `/image_raw` | 图像话题 |
| `exposure_time` | double | `10000.0` | 曝光时间 |
| `gain` | double | `0.0` | 相机增益 |
| `frame_rate` | double | `30.0` | 采集帧率 |
| `pixel_format` | string | `Mono8` | 像素格式 |

查看所有参数：

```bash
ros2 param list /hikrobot_camera
```

---

## 13. 动态修改参数

`camera_ip`、`serial_number`、`image_topic` 是启动参数，运行时修改会被拒绝，
需要修改配置后重启节点。`exposure_time`、`gain`、`frame_rate`、`pixel_format`
在相机已连接时可以动态修改；未连接时会被拒绝。

### 13.1 修改帧率

例如修改为 20 FPS：

```bash
ros2 param set /hikrobot_camera frame_rate 20.0
```

查看：

```bash
ros2 topic hz /image_raw
```

恢复为 30 FPS：

```bash
ros2 param set /hikrobot_camera frame_rate 30.0
```

---

### 13.2 修改曝光时间

例如：

```bash
ros2 param set /hikrobot_camera exposure_time 5000.0
```

程序会通过 MVS SDK 查询相机支持的参数范围，并在设置之前进行范围检查。
写入前会先关闭自动曝光，避免自动模式覆盖手动曝光。

---

### 13.3 修改增益

例如：

```bash
ros2 param set /hikrobot_camera gain 5.0
```

具体允许范围由当前相机的 MVS SDK 参数范围决定。
写入前会先关闭自动增益。

---

### 13.4 修改像素格式

当前版本支持：

```text
Mono8
```

例如：

```bash
ros2 param set /hikrobot_camera pixel_format Mono8
```

其他未实现的像素格式会被拒绝，并给出错误信息。

---

## 14. 参数合法性检查

程序在设置浮点型相机参数时，会首先通过 MVS SDK 查询参数范围：

```text
MV_CC_GetFloatValue()
```

然后进行范围检查。

只有合法参数才会通过：

```text
MV_CC_SetFloatValue()
```

写入相机。

因此非法曝光时间、增益或帧率不会直接发送给相机。

例如：

```bash
ros2 param set /hikrobot_camera frame_rate -1
```

程序会拒绝该参数：

```text
frame_rate must be greater than 0.
```

自动曝光/自动增益关闭失败、SDK 返回非零错误码，也会被节点拒绝并输出原因。

---

## 15. 图像消息格式

程序发布：

```text
sensor_msgs/msg/Image
```

话题：

```text
/image_raw
```

当前测试配置：

```text
width:     1440
height:    1080
encoding:  mono8
step:      1440
```

图像数据在发布前从 MVS SDK 图像缓冲区复制到 ROS 2 消息，因此消息不会依赖 MVS 缓冲区的后续生命周期。

---

## 16. 相机自动连接与状态监控

程序启动时会通过：

```text
MV_CC_EnumDevices()
```

枚举可用相机。

随后：

```text
MV_CC_CreateHandle()
        ↓
MV_CC_OpenDevice()
        ↓
配置相机参数
        ↓
MV_CC_StartGrabbing()
```

进入正常采集状态。

程序同时具有连接状态监控和重新枚举机制。当检测到相机不存在、或连续取流失败
超过阈值时，会释放当前相机句柄、停止抓取，并每 2 秒重新枚举设备；重连成功后
会重新应用 ROS 2 参数并恢复抓取。

> 注意：不同 USB/MVS 驱动环境在物理拔插后的 SDK 行为可能不同。当前版本已经实现重连机制，但实际硬件断线恢复仍建议在目标运行环境中进行单独验证。

---

## 17. 相机参数配置流程

启动时参数配置顺序：

```text
读取 ROS 2 参数
        ↓
发现相机
        ↓
选择相机
        ↓
CreateHandle
        ↓
OpenDevice
        ↓
关闭自动曝光
        ↓
关闭自动增益
        ↓
设置曝光时间
        ↓
设置增益
        ↓
设置帧率
        ↓
设置像素格式
        ↓
StartGrabbing
        ↓
发布 ROS Image
```

---

## 18. 故障排查

### 18.1 找不到相机

检查：

```bash
lsusb
```

确认 USB 相机已经连接。

如果 MVS 客户端（`/opt/MVS/bin/MVS`）正在运行，请先关闭它；相机被
MVS 客户端占用时，SDK 枚举可能返回 0，节点会一直显示找不到相机。

也可以使用 MVS 自带示例验证 SDK：

```bash
cd /opt/MVS/Samples/64/C++/General/GrabImage
./GrabImage
```

如果示例也无法获取图像，应优先检查 MVS SDK、USB 连接和相机硬件。

---

### 18.2 没有 `/image_raw`

检查：

```bash
ros2 node list
```

确认：

```text
/hikrobot_camera
```

然后：

```bash
ros2 topic list
```

确认：

```text
/image_raw
```

---

### 18.3 帧率不正确

检查：

```bash
ros2 param get /hikrobot_camera frame_rate
```

然后：

```bash
ros2 topic hz /image_raw
```

需要注意相机实际采集帧率与 ROS 2 话题测得的频率可能存在一定差异。

---

### 18.4 参数设置失败

查看节点终端中的错误信息。

常见原因：

- 参数超出相机支持范围
- 当前相机不支持该参数
- 相机尚未连接
- 参数名称错误
- MVS SDK 返回错误

---

## 19. MVS SDK

本项目依赖 Hikrobot MVS SDK。

SDK 提供相机设备枚举、设备打开、参数设置、图像采集和图像缓冲区管理等接口。

常用接口包括：

```text
MV_CC_EnumDevices()
MV_CC_CreateHandle()
MV_CC_OpenDevice()
MV_CC_StartGrabbing()
MV_CC_GetImageBuffer()
MV_CC_FreeImageBuffer()
MV_CC_StopGrabbing()
MV_CC_CloseDevice()
MV_CC_DestroyHandle()
MV_CC_GetFloatValue()
MV_CC_SetFloatValue()
MV_CC_SetEnumValue()
```

---

## 20. 当前测试结果

当前开发环境已经实际验证：

- Hikrobot USB 相机能够被 MVS SDK 正确发现
- 相机序列号能够正确读取
- 相机能够成功打开
- MVS SDK 能够正常获取图像
- ROS 2 节点能够正常启动
- `/image_raw` 能够正常发布
- 消息类型为 `sensor_msgs/msg/Image`
- 图像分辨率为 `1440 × 1080`
- 图像编码为 `mono8`
- 默认帧率约为 `30 Hz`
- `frame_rate` 可以运行时修改
- `exposure_time` 可以配置
- `gain` 可以配置
- 参数范围能够通过 MVS SDK 动态检查
- `pixel_format` 支持 `Mono8`
- ROS 2 Launch 和 YAML 配置能够正常工作

以上硬件结果以目标机实际相机为准；断线重连需要在硬件上按下一节步骤复核。

---

## 21. 项目启动命令汇总

### 编译

```bash
cd ~/assignment3-ROS2
colcon build --packages-select hikrobot_camera
```

### 加载工作空间

```bash
source install/setup.zsh
```

### 启动

```bash
ros2 launch hikrobot_camera camera.launch.py
```

### 查看节点

```bash
ros2 node list
```

### 查看图像话题

```bash
ros2 topic list | grep image
```

### 查看图像频率

```bash
ros2 topic hz /image_raw
```

### 查看图像

```bash
rqt_image_view
```

### 查看参数

```bash
ros2 param list /hikrobot_camera
```

### 修改帧率

```bash
ros2 param set /hikrobot_camera frame_rate 20.0
```

### 恢复 30 FPS

```bash
ros2 param set /hikrobot_camera frame_rate 30.0
```

---

## 22. 断线/重连手动验证

1. 连接相机并启动节点，确认 `/image_raw` 持续发布图像。
2. 拔掉 USB 线或断开网络相机的网线，观察节点日志出现“camera is no longer detected”或连续取流失败提示。
3. 重新插回相机，驱动会每 2 秒尝试重新枚举、连接并恢复抓取。
4. 如果恢复失败，先确认 MVS 自带示例能否重新发现相机，再查看节点终端中的 SDK 错误码。

---

## 23. 总结

本项目实现了一个基于 Hikrobot MVS SDK 的 ROS 2 工业相机驱动节点，实现了从相机设备发现、设备选择、参数配置、图像采集到 ROS 2 图像话题发布的完整流程。

核心数据流：

```text
Hikrobot Camera
      │
      ▼
  MVS SDK
      │
      ▼
CameraNode
      │
      ▼
MV_FRAME_OUT
      │
      ▼
sensor_msgs/msg/Image
      │
      ▼
 /image_raw
```

项目可以通过 ROS 2 Launch 和 YAML 参数文件进行配置和启动，适合在 Ubuntu 22.04 + ROS 2 Humble 环境下运行。
