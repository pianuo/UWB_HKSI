# UWB & GNSS Start Line Tracker

帆船运动起航线追踪系统，集成UWB高精度定位和GNSS定位。

## 功能特性

- **双地图显示**：左侧UWB定位地图，右侧GNSS定位地图（OpenStreetMap）
- **起航线检测**：自动检测Tag穿越Anchor 0和Anchor 1之间的起航线
- **实时指标**：
  - 当前速度 (m/s)
  - 到起航线距离 (m)
  - 预计到达时间 (s)
- **计时统计**：毫秒级精度计时

## 系统架构

```
┌─────────────────────┐                    ┌──────────────────────┐
│  GNSS Devices       │                    │  UWB Host Software   │
│  (3 Anchors)        │                    │  (Qorvo DWM3001CDK)  │
└─────────┬───────────┘                    └──────────┬───────────┘
          │ TCP (port 7799)                           │ UDP (port 5000)
          │ JSON: {client_id, lat, lng}               │ JSON: {TagID, X, Y, Z}
          ▼                                           │
┌─────────────────────┐                               │
│  TCP Server         │                               │
│  (tcp_server.py)    │                               │
└─────────┬───────────┘                               │
          │ ZMQ (port 5555)                           │
          ▼                                           ▼
┌───────────────────────────────────────────────────────────────┐
│                     Main Application                          │
│  ┌─────────────────────┐    ┌─────────────────────┐          │
│  │   UWB Map           │    │   GNSS Map          │          │
│  │   (Left)            │    │   (Right)           │          │
│  │   - UWB Tag位置     │    │   - GNSS Anchor位置 │          │
│  │   - 起航线(红)      │    │   - 起航线(红)      │          │
│  └─────────────────────┘    └─────────────────────┘          │
│                                                               │
│  [状态] [速度] [距离] [时间] [计时统计]                       │
└───────────────────────────────────────────────────────────────┘
```

## 安装

### 1. 安装Python依赖

```bash
pip install -r requirements.txt
```

或手动安装：
```bash
pip install tkintermapview numpy pyzmq
```

## 运行

### PowerShell
```powershell
cd C:\Users\cunyi\Desktop\cursor_code\UWB_HKSI
$env:PYTHONPATH="C:\Users\cunyi\Desktop\cursor_code\UWB_HKSI\src"
python -m uwb_line_tracker.main_app
```

### CMD
```cmd
cd C:\Users\cunyi\Desktop\cursor_code\UWB_HKSI
set PYTHONPATH=C:\Users\cunyi\Desktop\cursor_code\UWB_HKSI\src
python -m uwb_line_tracker.main_app
```

## 配置说明

### UWB上位机软件配置
- **远程主机IP**：`127.0.0.1`
- **远程主机端口**：`5000`
- **数据格式**：
```json
{
    "Command": "UpLink",
    "TagID": 0,
    "X": 1.089,
    "Y": 1.056,
    "Z": 1.539
}
```

### GNSS设备配置
- **TCP服务器端口**：`7799`
- **数据格式**（每行一个JSON，换行符分隔）：
```json
{"client_id": 0, "lat": 22.31930, "lng": 114.16940, "alt": 10.0}
{"client_id": 1, "lat": 22.31930, "lng": 114.16965, "alt": 10.0}
{"client_id": 2, "lat": 22.31945, "lng": 114.16952, "alt": 10.0}
```

其中：
- `client_id`: 0, 1, 2 对应三个Anchor
- `lat`: 纬度
- `lng`: 经度
- `alt`: 高度（可选）

### UWB Anchor配置
点击 **"Configure UWB Anchors"** 按钮，输入3个UWB anchor的XYZ坐标（米）：
- **Anchor 0**：起航线端点1
- **Anchor 1**：起航线端点2
- **Anchor 2**：坐标系参考点

## 坐标转换

系统使用基于3个共位anchor的刚体变换，将UWB局部坐标转换为GPS全局坐标：

1. **构建坐标基**：在UWB和ENU坐标系中分别构建正交坐标基
2. **计算旋转矩阵**：R = E × U^T
3. **计算平移向量**：t = p₀^E - R × p₀^U
4. **应用刚体变换**：p^E = R × p^U + t
5. **ENU转GPS**：将ENU坐标转换为经纬度

详见 `src/uwb2llh.md`

## 界面说明

| 元素 | 说明 |
|------|------|
| **UWB Map (左)** | 显示UWB定位结果（转换到GPS坐标后） |
| **GNSS Map (右)** | 显示直接GNSS定位结果（1Hz更新） |
| **Start Line Status** | 红色=未穿越，绿色=已穿越 |
| **Speed** | 当前速度 (m/s) |
| **Distance to Start Line** | 到起航线的垂直距离 (m) |
| **Time to Start Line** | 预计到达时间 = 距离/速度 (s) |
| **Timing Statistics** | 开始/结束时间和持续时间（毫秒精度） |

## 按钮功能

| 按钮 | 功能 |
|------|------|
| **Configure UWB Anchors** | 配置UWB anchor的XYZ坐标 |
| **Reset Timing** | 重置计时统计 |
| **Start Timing** | 开始计时 |
| **Recalibrate** | 手动触发坐标校准 |

## 故障排除

### 地图不显示
```bash
pip install tkintermapview
```
确保有网络连接

### 无UWB数据
- 检查UDP端口5000是否被防火墙阻止
- 确认UWB上位机软件配置正确

### 无GNSS数据
- 检查TCP端口7799是否可访问
- 确认GNSS设备发送格式正确

### 校准失败
- 确保3个GNSS anchor都有数据
- 确保3个anchor不共线
- 确认UWB anchor坐标正确

## 文件结构

```
src/uwb_line_tracker/
├── __init__.py           # 包初始化
├── main_app.py           # 主应用程序
├── coordinate_transform.py # UWB到GPS坐标转换
├── udp_receiver.py       # UWB UDP数据接收
├── gnss_receiver.py      # GNSS ZMQ数据接收
├── tcp_server.py         # GNSS TCP服务器
├── config.py             # 配置参数
├── models.py             # 数据模型
└── ds_twr.py             # DS-TWR算法
```
