# Anchor Self-Calibration Guide

## 概述

Anchor 自标定功能允许系统自动从 UWB 硬件获取 anchor 之间的测距数据，并使用 MDS (Multi-Dimensional Scaling) 算法计算 anchor 的相对位置。

## 使用方法

### 1. 打开标定对话框

点击主界面上的 **"Anchor Self-Calibration"** 按钮。

### 2. 开始数据采集

1. 确保三个 anchor 都已开机并连接
2. 点击 **"Start Collection"** 按钮
3. 系统将自动从 UDP 端口 8080 接收 anchor 之间的测距数据
4. 观察 "Inter-Anchor Distances" 表格中的数据更新

### 3. 执行标定

当收集到足够的数据后（每个 anchor 对至少一个测量值）：
1. 点击 **"Calibrate"** 按钮
2. 系统将使用 MDS 算法计算 anchor 位置
3. 结果将显示在 "Calibrated Anchor Positions" 表格中

### 4. 确认应用

点击 **"Confirm"** 按钮将标定结果应用到系统。

## UDP 数据格式

系统需要从 UDP 端口 8080 接收以下格式的数据：

### 格式 1: 单个 Anchor 测距

```json
{
    "Command": "AnchorRange",
    "AnchorI": 0,
    "AnchorJ": 1,
    "Distance": 25000
}
```

- `AnchorI`: 第一个 anchor 的 ID (0, 1, 或 2)
- `AnchorJ`: 第二个 anchor 的 ID (0, 1, 或 2)
- `Distance`: 距离值（单位：毫米）

### 格式 2: 多个测距数据

```json
{
    "Command": "RangeData",
    "Ranges": [
        {"I": 0, "J": 1, "Distance": 25000},
        {"I": 0, "J": 2, "Distance": 30000},
        {"I": 1, "J": 2, "Distance": 28000}
    ]
}
```

### 格式 3: 手动输入

如果硬件不支持自动发送 anchor 之间的测距数据，您可以：
1. 使用测量工具测量 anchor 之间的距离
2. 在 "Manual Distance Entry" 区域输入距离（单位：毫米）
3. 点击 **"Add"** 按钮添加数据

## 算法说明

### MDS (Multi-Dimensional Scaling)

1. 从距离矩阵构建中心化矩阵
2. 使用 SVD 分解获取相对位置
3. 将坐标系原点设置为 A0

### 角度旋转

1. 旋转坐标系，使 A1 位于正 X 轴
2. 确保 A2 位于第一象限（正 Y 值）

## 故障排除

### 问题：显示 "Insufficient data for calibration"

**原因**：系统没有收到足够的 anchor 间测距数据。

**解决方案**：
1. 确保三个 anchor 都已开机
2. 检查 UDP 端口 8080 是否正在接收数据
3. 确认硬件发送的数据格式正确
4. 尝试手动输入距离数据

### 问题：UDP 接收器未运行

**解决方案**：
1. 确保主程序已启动
2. 检查 UDP 端口 8080 是否被占用
3. 查看控制台输出中的 UDP 状态信息

### 问题：数据格式不正确

**解决方案**：
1. 检查硬件发送的 JSON 格式是否正确
2. 确认 `Command` 字段为 "AnchorRange" 或 "RangeData"
3. 确认距离单位为毫米

## 技术细节

- **端口**: UDP 8080
- **数据格式**: JSON
- **距离单位**: 毫米（转换为米后用于计算）
- **最小要求**: 每个 anchor 对至少一个测量值（A0-A1, A0-A2, A1-A2）

