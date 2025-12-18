# UWB 帆船冲线定位系统 - 技术文档

## 项目概述

本项目是一个基于 **UWB (Ultra-Wideband)** 技术的帆船运动定位系统，用于精确测量和分析帆船在"冲线"关键阶段的时机、位置和调整。系统使用 **Qorvo DWM3001CDK** 开发套件，通过 **DS-TWR (Double-Sided Two-Way Ranging)** 方法实现厘米级定位精度和纳秒级计时。

### 核心目标
- **高精度定位**：厘米级（<10cm）位置测量
- **高频率更新**：100Hz 实时数据流
- **精确计时**：纳秒级时间戳，<1ms 延迟
- **虚拟起点线检测**：自动识别帆船跨越两个锚点形成的虚拟起点线

---

## 系统架构

```
┌─────────────────┐         TCP/JSON         ┌──────────────────┐
│   Simulator     │ ────────────────────────> │   Visualizer     │
│  (信号生成器)    │    (127.0.0.1:8765)      │   (可视化终端)   │
│                 │                           │                  │
│ - 圆形轨迹模拟  │                           │ - 实时轨迹显示  │
│ - DS-TWR 信号   │                           │ - 信息面板      │
│ - 噪声注入      │                           │ - 历史回放      │
└─────────────────┘                           └──────────────────┘
                                                        │
                                                        ▼
                                               ┌──────────────────┐
                                               │  History Store   │
                                               │  (历史数据存储)  │
                                               │  frames.jsonl    │
                                               └──────────────────┘
```

---

## 核心模块详解

### 1. `config.py` - 配置管理模块

**功能**：集中管理系统的所有配置参数

**主要配置项**：

```python
# 物理常量
SPEED_OF_LIGHT_M_S = 299_792_458.0  # 光速（米/秒）

# 锚点配置（定义虚拟起点线的两个端点）
ANCHORS = [
    {"id": "A", "position": (0.0, 0.0, 0.0)},    # 锚点A
    {"id": "B", "position": (25.0, 0.0, 0.0)},   # 锚点B（25米外）
]

# 模拟参数
SIMULATION = {
    "update_rate_hz": 100,              # 更新频率：100Hz
    "distance_noise_std_m": 0.015,      # 距离噪声：1.5cm标准差
    "circle_radius_m": 18.0,             # 圆形轨迹半径：18米
    "circle_period_s": 22.0,            # 一圈周期：22秒
    "radial_noise_std_m": 0.45,         # 径向噪声
    "angular_noise_std_deg": 1.2,       # 角度噪声
}
```

**设计原理**：
- **单一配置源**：所有参数集中管理，便于调试和部署
- **物理单位明确**：使用标准SI单位（米、秒、纳秒）
- **可扩展性**：新增参数只需在此添加，不影响其他模块

---

### 2. `models.py` - 数据模型模块

**功能**：定义系统的核心数据结构

**核心类**：

#### `Vector3`
```python
Vector3 = Tuple[float, float, float]  # (x, y, z) 坐标
```

#### `Anchor` - 锚点
```python
@dataclass
class Anchor:
    id: str              # 锚点标识（如 "A", "B"）
    position: Vector3    # 三维坐标（米）
```
**作用**：表示固定在浮标上的UWB基站，用于接收和发送测距信号

#### `TagState` - 标签状态
```python
@dataclass
class TagState:
    id: str              # 标签标识
    true_position: Vector3    # 真实位置（模拟器用）
    velocity: Vector3        # 速度向量（m/s）
```
**作用**：表示安装在帆船上的UWB标签的当前状态

#### `RangeMeasurement` - 测距测量
```python
@dataclass
class RangeMeasurement:
    anchor_id: str
    tag_id: str
    tof_ns: float              # 飞行时间（纳秒）
    distance_m: float          # 测量距离（米）
    variance_m2: float         # 距离方差
    round_trip_ns: float       # 往返时间
    reply_time_ns: float       # 响应延迟
```
**作用**：存储一次DS-TWR测距的完整结果

#### `Frame` - 数据帧
```python
@dataclass
class Frame:
    frame_id: int
    timestamp_ns: int          # 时间戳（纳秒）
    anchors: Tuple[Anchor, ...]
    tag: TagState
    measurements: Tuple[RangeMeasurement, ...]
    line_crossing: dict | None  # 冲线事件（如有）
```
**作用**：一个完整的数据帧，包含所有锚点、标签状态和测量结果

**设计原理**：
- **不可变数据**：使用 `@dataclass` 和 `Tuple`，避免意外修改
- **类型安全**：明确的类型注解，便于IDE检查和调试
- **序列化支持**：`to_payload()` 方法将数据转换为JSON格式

---

### 3. `ds_twr.py` - DS-TWR 算法模块

**功能**：实现双边双向测距（DS-TWR）的核心算法

#### 3.1 DS-TWR 原理

**DS-TWR (Double-Sided Two-Way Ranging)** 是一种高精度测距方法，通过四次信号交换消除时钟偏差：

```
时间线：
T0: Tag 发送 Poll 信号
T1: Anchor 接收 Poll
T2: Anchor 发送 Response
T3: Tag 接收 Response
T4: Tag 发送 Final
T5: Anchor 接收 Final

距离计算：
ToF = [(T1-T0) * (T5-T2) - (T3-T2) * (T4-T1)] / [(T1-T0) + (T5-T2) + (T3-T2) + (T4-T1)]
距离 = ToF * c / 2  (c = 光速)
```

**优势**：
- **消除时钟偏差**：不需要Tag和Anchor时钟同步
- **高精度**：厘米级精度（<10cm）
- **抗干扰**：UWB信号抗多径干扰能力强

#### 3.2 核心函数

##### `simulate_ds_twr()` - 模拟DS-TWR测距
```python
def simulate_ds_twr(anchor, tag, noise_std_m, responder_delay_ns, clock_drift_ns):
    true_distance = euclidean_distance(anchor.position, tag.true_position)
    noisy_distance = true_distance + rng.gauss(0.0, noise_std_m)  # 添加高斯噪声
    tof_ns = distance_to_tof_ns(noisy_distance) + clock_drift_ns
    return RangeMeasurement(...)
```

**原理**：
1. 计算真实距离：`√[(x₁-x₂)² + (y₁-y₂)² + (z₁-z₂)²]`
2. 添加测量噪声：模拟真实UWB设备的测量误差
3. 转换为飞行时间：`ToF = 距离 / 光速`
4. 添加时钟漂移：模拟硬件时钟不完美

##### `estimate_tag_position()` - 三角定位
```python
def estimate_tag_position(anchors, measurements):
    # 使用两个锚点的距离测量，通过圆相交求解Tag位置
    return _circle_intersection(anchor_a, anchor_b, dist_a, dist_b)
```

**原理 - 三角测量（Trilateration）**：

给定两个锚点 A、B 和它们到Tag的距离 d₁、d₂：

```
以A为圆心，d₁为半径的圆： (x-xₐ)² + (y-yₐ)² = d₁²
以B为圆心，d₂为半径的圆： (x-xᵦ)² + (y-yᵦ)² = d₂²
```

两个圆的交点就是Tag的可能位置（通常有两个解，根据`prefer_positive_side`选择）。

**数学推导**：
1. 将坐标系旋转，使AB在x轴上
2. 设Tag坐标为(x, y)，A在原点，B在(d, 0)
3. 解方程组得到：
   ```
   x = (d₁² - d₂² + d²) / (2d)
   y = ±√(d₁² - x²)
   ```
4. 旋转回原坐标系

##### `detect_line_crossing()` - 冲线检测
```python
def detect_line_crossing(anchor_a, anchor_b, previous_point, current_point):
    side_prev = side_of_line(anchor_a, anchor_b, previous_point)
    side_curr = side_of_line(anchor_a, anchor_b, current_point)
    return side_prev * side_curr < 0  # 符号相反表示跨越
```

**原理**：
- 使用**有向面积**判断点在线的哪一侧
- 如果前一点和当前点在线的不同侧，说明跨越了线
- 公式：`side = (bx-ax)(py-ay) - (by-ay)(px-ax)`
  - `side > 0`：点在左侧
  - `side < 0`：点在右侧
  - `side = 0`：点在线上

##### `signed_distance_to_line()` - 带符号距离
```python
def signed_distance_to_line(anchor_a, anchor_b, point):
    side = side_of_line(anchor_a, anchor_b, point)
    line_length = math.hypot(bx-ax, by-ay)
    return side / line_length  # 垂直距离
```

**用途**：实时显示Tag距离虚拟起点线的距离，正负表示在线的哪一侧

---

### 4. `simulator.py` - 信号模拟器模块

**功能**：模拟UWB锚点和标签，生成DS-TWR测距数据

#### 4.1 `RegattaSimulator` - 帆船运动模拟器

**运动模型**：圆形轨迹 + 噪声扰动

```python
def _update_tag_state(self, dt):
    # 1. 基础圆形运动
    self._theta = (self._theta + self._omega * dt) % (2 * math.pi)
    x = cx + radius * cos(theta)
    y = cy + radius * sin(theta)
    
    # 2. 添加噪声扰动
    angular_noise = gauss(0, angular_noise_std)  # 角度噪声
    radial_noise = gauss(0, radial_noise_std)     # 径向噪声
    gust_x = gauss(0, gust_noise_std)            # 阵风扰动X
    gust_y = gauss(0, gust_noise_std)            # 阵风扰动Y
    
    # 3. 合成最终位置
    x = cx + (radius + radial_noise) * cos(theta + angular_noise) + gust_x
    y = cy + (radius + radial_noise) * sin(theta + angular_noise) + gust_y
```

**设计原理**：
- **物理真实性**：模拟帆船在起跑线前的调姿动作（近似圆形）
- **随机扰动**：三层噪声模拟真实环境
  - **角度噪声**：模拟舵手操作不完美
  - **径向噪声**：模拟风速变化
  - **阵风扰动**：模拟突发风况
- **无限循环**：`theta % (2π)` 确保轨迹持续循环

#### 4.2 `SimulatorServer` - TCP服务器

**功能**：通过TCP连接向客户端广播数据帧

**通信协议**：
- **格式**：JSON Lines（每行一个JSON对象）
- **端口**：8765（可配置）
- **更新率**：100Hz（每10ms一帧）

```python
async def _broadcast_loop(self):
    interval = 1.0 / self.update_rate_hz  # 0.01秒
    while True:
        frame = self.simulator.generate_frame()
        payload = json.dumps(frame.to_payload()) + "\n"
        for writer in self._clients:
            writer.write(payload.encode("utf-8"))
        await asyncio.sleep(interval)
```

**设计原理**：
- **异步I/O**：使用`asyncio`实现高并发
- **非阻塞**：多个客户端可同时连接
- **轻量级协议**：JSON Lines格式简单，易于调试

---

### 5. `terminal.py` - 命令行终端模块

**功能**：接收模拟器数据，在终端显示实时信息

**主要功能**：
1. **TCP客户端**：连接到模拟器服务器
2. **实时显示**：打印锚点位置、Tag估计位置、时间戳
3. **历史记录**：将数据写入`history/frames.jsonl`
4. **回放功能**：`--replay` 参数查看历史数据

**输出示例**：
```
Anchors:
  - A: (0.00, 0.00, 0.00) m
  - B: (25.00, 0.00, 0.00) m

[     0.0 ms] Frame 00234 | Tag ≈ (  0.469,   5.038) m
[    12.7 ms] Frame 00235 | Tag ≈ (  0.509,   5.021) m
[    28.6 ms] Frame 00236 | Tag ≈ (  0.615,   4.959) m
```

---

### 6. `visualizer.py` - 可视化GUI模块

**功能**：提供图形化界面，实时可视化锚点、Tag轨迹和冲线事件

#### 6.1 架构组件

##### `SimulatorController` - 模拟器控制器
```python
class SimulatorController:
    def start(self):
        # 在后台启动模拟器进程
        self._process = subprocess.Popen(
            [sys.executable, "-m", "uwb_line_tracker.simulator"]
        )
```

**功能**：管理模拟器进程的生命周期（启动/停止）

##### `FrameReceiver` - 帧接收器
```python
class FrameReceiver(threading.Thread):
    def run(self):
        # 在后台线程中持续接收TCP数据
        sock = socket.create_connection((host, port))
        while True:
            frame = json.loads(stream.readline())
            self.frame_queue.put(frame)
```

**功能**：在独立线程中接收数据，避免阻塞GUI主线程

##### `VisualizerApp` - 主应用
```python
class VisualizerApp:
    def __init__(self):
        # 1. 创建画布
        self.canvas = tk.Canvas(...)
        
        # 2. 创建控制按钮
        self.start_button = ttk.Button(..., command=self.start_simulator)
        self.history_button = ttk.Button(..., command=self.show_history_window)
        
        # 3. 创建信息显示
        self.tag_var = tk.StringVar()  # Tag位置
        self.speed_var = tk.StringVar()  # 速度
        self.distance_var = tk.StringVar()  # 距线距离
        self.crossing_var = tk.StringVar()  # 冲线时间
```

#### 6.2 可视化原理

##### 坐标转换
```python
def _world_to_canvas(self, x, y):
    # 世界坐标 -> 画布像素坐标
    scale = min((width - padding) / span_x, (height - padding) / span_y)
    px = padding + (x - min_x) * scale
    py = height - (padding + (y - min_y) * scale)  # Y轴翻转
```

**原理**：
- **缩放**：根据世界坐标范围自动计算缩放比例
- **平移**：将世界坐标原点映射到画布左上角
- **Y轴翻转**：屏幕坐标系Y向下，世界坐标系Y向上

##### 实时绘制
```python
def _redraw_canvas(self):
    # 1. 绘制虚拟起点线（蓝色）
    canvas.create_line(ax, ay, bx, by, fill="#38bdf8")
    
    # 2. 绘制锚点（青色圆点）
    for anchor in anchors:
        canvas.create_oval(px-6, py-6, px+6, py+6, fill="#22d3ee")
    
    # 3. 绘制Tag轨迹（紫色线条）
    canvas.create_line(*path_coords, fill="#a855f7", smooth=True)
    
    # 4. 绘制当前Tag位置（橙色圆点）
    canvas.create_oval(px-10, py-10, px+10, py+10, fill="#f97316")
```

**更新机制**：
- **定时轮询**：每50ms检查一次数据队列
- **增量更新**：只重绘变化的部分
- **轨迹缓存**：使用`deque`保存最近2000个点

---

### 7. `history.py` - 历史数据管理模块

**功能**：持久化存储和检索历史数据

**存储格式**：JSON Lines（`.jsonl`）
```json
{"frame_id": 123, "timestamp_ns": 1764580813759045700, "position_xy": [142.716, 23.049], "distance_to_line_m": 5.23, "line_crossing": null}
{"frame_id": 124, "timestamp_ns": 1764580813779044600, "position_xy": [142.790, 23.222], "distance_to_line_m": 4.87, "line_crossing": {"timestamp_ns": 1764580813779044600}}
```

**设计原理**：
- **追加写入**：使用`"a"`模式，性能高
- **行分隔**：每行一个JSON，易于流式处理
- **可扩展**：新增字段不影响旧数据

---

## 数据流图

```
┌─────────────────────────────────────────────────────────────┐
│                    Simulator (模拟器)                         │
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │ 运动模型     │ -> │ DS-TWR模拟   │ -> │ 数据帧生成    │  │
│  │ (圆形轨迹)   │    │ (测距计算)   │    │ (Frame对象)   │  │
│  └──────────────┘    └──────────────┘    └──────────────┘  │
│         │                    │                    │          │
│         └────────────────────┴────────────────────┘          │
│                              │                               │
│                    ┌──────────▼──────────┐                   │
│                    │  TCP Server (8765)   │                   │
│                    └──────────┬──────────┘                   │
└───────────────────────────────┼───────────────────────────────┘
                                 │ JSON Lines
                                 ▼
┌─────────────────────────────────────────────────────────────┐
│              Visualizer / Terminal (接收端)                  │
│                                                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐  │
│  │ TCP Client   │ -> │ 数据解析     │ -> │ 位置估计     │  │
│  │ (接收帧)     │    │ (JSON->Frame)│    │ (三角定位)   │  │
│  └──────────────┘    └──────────────┘    └──────────────┘  │
│         │                    │                    │          │
│         └────────────────────┴────────────────────┘          │
│                              │                               │
│                    ┌──────────▼──────────┐                   │
│                    │  可视化/显示        │                   │
│                    │  (GUI/CLI)          │                   │
│                    └──────────┬──────────┘                   │
│                              │                               │
│                    ┌──────────▼──────────┐                   │
│                    │  History Store      │                   │
│                    │  (frames.jsonl)     │                   │
│                    └─────────────────────┘                   │
└─────────────────────────────────────────────────────────────┘
```

---

## 关键技术原理

### 1. DS-TWR 测距原理

**问题**：如何精确测量两个设备之间的距离？

**解决方案**：DS-TWR（双边双向测距）

**步骤**：
1. **Poll阶段**：Tag发送信号，记录时间T₀
2. **Response阶段**：Anchor收到后立即回复，Tag记录时间T₁
3. **Final阶段**：Tag发送最终信号，Anchor记录时间T₂

**距离计算**：
```
ToF = (Tround1 × Tround2 - Treply1 × Treply2) / (Tround1 + Tround2 + Treply1 + Treply2)
距离 = ToF × c / 2
```

**优势**：
- 消除时钟偏差（不需要同步）
- 高精度（厘米级）
- 实时性好

### 2. 三角定位（Trilateration）

**问题**：如何从距离测量得到位置？

**解决方案**：两个圆的交点

给定：
- 锚点A位置：(xₐ, yₐ)
- 锚点B位置：(xᵦ, yᵦ)
- Tag到A的距离：d₁
- Tag到B的距离：d₂

求解Tag位置(x, y)：

```
(x - xₐ)² + (y - yₐ)² = d₁²  ... (1)
(x - xᵦ)² + (y - yᵦ)² = d₂²  ... (2)
```

展开并相减：
```
2(xᵦ - xₐ)x + 2(yᵦ - yₐ)y = d₁² - d₂² - xₐ² + xᵦ² - yₐ² + yᵦ²
```

这是一个线性方程，结合(1)或(2)可求解。

### 3. 冲线检测算法

**问题**：如何判断Tag是否跨越了虚拟起点线？

**解决方案**：有向面积法

对于点P和线段AB，计算：
```
side = (Bx - Ax)(Py - Ay) - (By - Ay)(Px - Ax)
```

- `side > 0`：P在AB左侧
- `side < 0`：P在AB右侧
- `side = 0`：P在AB上

**冲线判断**：
如果前一点和当前点在线的不同侧，说明跨越了线：
```python
if side_previous * side_current < 0:
    # 冲线！
```

---

## 性能指标

| 指标 | 目标值 | 实现值 |
|------|--------|--------|
| 定位精度 | <10cm | ~1.5cm (模拟) |
| 更新频率 | 100Hz | 100Hz |
| 时间精度 | <1ms | 纳秒级 |
| 延迟 | 实时 | <10ms |

---

## 部署到真实硬件

当使用真实的Qorvo DWM3001CDK硬件时，只需替换数据源：

### 当前（模拟器）：
```python
# simulator.py
measurements = simulate_ds_twr(anchor, tag, ...)  # 模拟数据
```

### 真实硬件：
```python
# hardware_adapter.py
measurements = read_dwm3001_ranging(anchor_id, tag_id)  # 真实数据
```

**接口保持不变**：`RangeMeasurement` 对象格式一致，终端和可视化器无需修改。

---

## 总结

本项目实现了一个完整的UWB定位系统原型，包括：

1. **信号模拟**：模拟UWB DS-TWR测距过程
2. **算法实现**：三角定位、冲线检测
3. **数据可视化**：实时轨迹显示、历史回放
4. **系统架构**：模块化设计，易于扩展

**核心优势**：
- ✅ 高精度（厘米级）
- ✅ 实时性好（100Hz）
- ✅ 易于部署（只需替换数据源）
- ✅ 完整可视化（GUI + CLI）

**适用场景**：
- 帆船比赛起跑线检测
- 其他需要精确位置测量的运动项目
- UWB定位系统原型开发

