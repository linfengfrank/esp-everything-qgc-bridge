# QGroundControl MAVLink WiFi Bridge

本项目已经增加了一个简单的 MAVLink UDP 桥接功能，使 QGroundControl 可以通过 ESP32-S3 的 WiFi 链路看到 PX4。

## 通信链路

```text
QGroundControl / Laptop
        ↑↓  UDP MAVLink
        │   QGC listen port: 14550
        │   ESP32 local/source port: 8888
        │
ESP32-S3 companion computer
        ↑↓  UART MAVLink, 921600 baud
        │
PX4 flight controller
```

ESP32 仍然保留原来的 Python GCS 自定义 UDP 协议：

```text
ESP32 → Python GCS: UDP 5005 telemetry
Python GCS → ESP32: UDP 5006 command
ESP32 → Laptop: UDP 5007 ToF debug
```

QGC MAVLink bridge 与这些端口不冲突。

## 已修改的代码

主要修改在：

```text
main/mavlink_task.c
main/mavlink_task.h
```

核心设计：

1. `mavlink_task.c` 仍然是唯一读取 PX4 UART 的任务。
2. 从 PX4 UART 读到的原始 MAVLink bytes 会同时：
   - 转发到 QGroundControl: `CONFIG_HOST_IPV4_ADDR:CONFIG_MAVROS_BRIDGE_OUT_PORT`
   - 被 ESP32 内部解析，用于更新位置、姿态和 heartbeat 状态。
3. QGroundControl 发到 ESP32 UDP 端口 `CONFIG_MAVROS_BRIDGE_IN_PORT` 的 MAVLink bytes 会被写回 PX4 UART。
4. 这样避免了两个 task 同时读取 UART 导致 MAVLink 字节流被破坏的问题。

## 默认配置

在 `sdkconfig` / `Kconfig.projbuild` 中：

```text
CONFIG_HOST_IPV4_ADDR="192.168.50.160"
CONFIG_MAVROS_BRIDGE_IN_PORT=8888
CONFIG_MAVROS_BRIDGE_OUT_PORT=14550
```

含义：

```text
ESP32 sends PX4 MAVLink to laptop/QGC: 192.168.50.160:14550
ESP32 listens for QGC MAVLink on local UDP port: 8888
```

请确认 `CONFIG_HOST_IPV4_ADDR` 是运行 QGroundControl 的电脑 IP。

## QGroundControl 设置方法

推荐先关闭 QGC 自动连接，然后手动增加 UDP link：

```text
Type: UDP
Listening Port: 14550
Target Host: ESP32 的 IP 地址
Target Port: 8888
```

很多情况下，只要 ESP32 主动向电脑的 `14550` 端口发送 PX4 heartbeat，QGC 也会自动发现车辆。但是手动配置 UDP link 更稳定。

## PX4 参数检查

ESP32 通过 UART 与 PX4 通信，默认参数是：

```text
ESP32 UART: UART1
ESP32 TX: GPIO5
ESP32 RX: GPIO4
Baudrate: 921600
```

PX4 对应 TELEM 端口需要配置为 MAVLink，并且 baudrate 要匹配。例如如果接在 TELEM2：

```text
MAV_2_CONFIG = TELEM2
MAV_2_MODE = Onboard
SER_TEL2_BAUD = 921600
```

修改参数后重启 PX4。

## 测试步骤

1. 先拆桨测试，不要带桨调试 MAVLink / QGC。
2. 确认 PX4 通过 USB 能被 QGC 正常识别。
3. 确认 ESP32 串口日志显示：

```text
UART1 ready: TX=GPIO5 RX=GPIO4 @ 921600 baud
QGC MAVLink bridge enabled: PX4 UART ↔ UDP <laptop_ip>:14550, local UDP 8888
Telemetry valid
```

4. 打开 QGroundControl，确认电脑防火墙允许 UDP 14550。
5. 如果 QGC 仍然看不到飞机，检查：
   - `CONFIG_HOST_IPV4_ADDR` 是否是电脑当前 IP
   - 电脑和 ESP32 是否在同一个 WiFi 网络
   - PX4 TELEM baudrate 是否为 921600
   - TX/RX 是否交叉连接，GND 是否共地
   - QGC UDP link 的 target port 是否为 8888

## 注意事项

这个 bridge 是 MAVLink 透明转发。QGroundControl 发出的 ARM、mode change、parameter request 等 MAVLink 消息会被转发给 PX4。实机测试时请先拆桨，并避免同时从 Python GCS 和 QGC 发送互相冲突的飞行命令。
