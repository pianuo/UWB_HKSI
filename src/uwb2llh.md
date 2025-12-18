# UWB 坐标转换为 GPS 经纬度算法说明

## A. 输入数据

已知 anchor 数据（对应同一物理点）：

### UWB（局部坐标，单位 m）

$$p_i^U = \begin{bmatrix} X_i \\ Y_i \\ Z_i \end{bmatrix}, \quad i = 0, 1, 2$$

### GNSS（全球坐标）

$$(\varphi_i, \lambda_i, h_i)$$

其中：
- $\varphi_i$ = 纬度
- $\lambda_i$ = 经度  
- $h_i$ = 高度

### Tag（UWB 输出）

$$p_{tag}^U = \begin{bmatrix} X \\ Y \\ Z \end{bmatrix}$$

---

## B. GNSS → 局部 ENU 坐标系（线性化）

### 数学动机

- 经纬度 **不能直接旋转**
- 必须转成 **欧氏空间（米）**

### 做法

1. 选 anchor0 的 GNSS 作为参考点：$(\varphi_0, \lambda_0, h_0)$

2. 将所有 anchor GNSS 转为 ENU：

$$p_i^E = \begin{bmatrix} E_i \\ N_i \\ U_i \end{bmatrix}$$

**转换本质：** LLA → ECEF → ENU  
（你可以视为一个确定的坐标投影步骤）

---

## C. 在 UWB 坐标系中构造正交基

### 1️⃣ 定义 X 轴（anchor0 → anchor1）

$$\hat{x}^U = \frac{p_1^U - p_0^U}{\|p_1^U - p_0^U\|}$$

✔ 与你的认知完全一致："基站0和1为 x 轴"

### 2️⃣ 定义 Z 轴（右手系法向量）

$$\hat{z}^U = \frac{(p_1^U - p_0^U) \times (p_2^U - p_0^U)}{\|(p_1^U - p_0^U) \times (p_2^U - p_0^U)\|}$$

**数学意义：**
- anchor0, 1, 2 定义一个平面
- Z 是这个平面的法向量
- 保证 **右手系**

### 3️⃣ 定义 Y 轴（垂直于 X）

$$\hat{y}^U = \hat{z}^U \times \hat{x}^U$$

✔ 符合你说的："y 可能是垂直于 x 轴方向"

### 4️⃣ 得到 UWB 坐标基矩阵

$$U = \begin{bmatrix} \hat{x}^U & \hat{y}^U & \hat{z}^U \end{bmatrix}$$

---

## D. 在 ENU 坐标系中构造对应坐标基

**完全同构，只是把 UWB 换成 ENU**

### 1️⃣ ENU 的 X 轴（anchor0 → anchor1）

$$\hat{x}^E = \frac{p_1^E - p_0^E}{\|p_1^E - p_0^E\|}$$

### 2️⃣ ENU 的 Z 轴

$$\hat{z}^E = \frac{(p_1^E - p_0^E) \times (p_2^E - p_0^E)}{\|(p_1^E - p_0^E) \times (p_2^E - p_0^E)\|}$$

### 3️⃣ ENU 的 Y 轴

$$\hat{y}^E = \hat{z}^E \times \hat{x}^E$$

### 4️⃣ 得到 ENU 坐标基矩阵

$$E = \begin{bmatrix} \hat{x}^E & \hat{y}^E & \hat{z}^E \end{bmatrix}$$

---

## E. 求旋转矩阵 $R$

### 数学原理（核心）

- 同一"物理坐标架"
- 在 UWB 和 ENU 中的表达不同
- **坐标变换 = 坐标基对齐**

### 公式

$$R = E \cdot U^T$$

### 性质检查（工程强烈建议）

- $R^T R = I$
- $\det(R) \approx +1$

---

## F. 求平移向量 $t$

### 数学原理

anchor0 在两个坐标系中是同一物理点

### 公式

$$t = p_0^E - R p_0^U$$

---

## G. Tag：UWB → ENU

### 刚体变换公式

$$p_{tag}^E = R p_{tag}^U + t$$

此时你已经得到了：**tag 在局部 ENU（米）下的位置**

---

## H. ENU → GNSS（lat / lng / alt）

### 数学原理

- ENU 是参考点的切平面
- 通过反投影回全球坐标

### 做法

$$(E, N, U) + (\varphi_0, \lambda_0, h_0) \Rightarrow (\varphi_{tag}, \lambda_{tag}, h_{tag})$$

**转换路径：** ENU → ECEF → LLA

---

## 总结

整个转换流程：

```
UWB坐标 → (刚体变换) → ENU坐标 → (坐标投影) → GPS经纬度
```

关键步骤：
1. 基于3个anchor点构造UWB和ENU的正交坐标系
2. 通过坐标基对齐求解旋转矩阵 $R$
3. 通过anchor0求解平移向量 $t$
4. 应用刚体变换：$p^E = Rp^U + t$
5. ENU转回GPS经纬度
