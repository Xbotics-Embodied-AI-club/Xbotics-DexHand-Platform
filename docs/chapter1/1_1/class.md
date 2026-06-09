# 灵巧手线上实习第一个月

# M1 基础夯实月｜学习资料准备清单（文档 \&amp; 视频）

> 定位：**零基础也能跟上的“机器人 \+ 灵巧手 \+ 仿真”入门资料包**
> 原则：**不推公式、不堆论文，重结构理解 \+ 能动手**
> 
> 

---

## 第一个月用到的学习资料文档

\[Linker  Hand L6 产品手册\.pdf\]

\[Linker  Hand O6 产品手册\.pdf\]



\[Linker  Hand L20产品手册\.pdf\]

\[线上实习第一周\.pdf\]



\[线上实习第二周——URDF文件核心参数与结构详解\.pdf\]

### 主题：机器人基础 \+ 灵巧手整体认知 

---

## 一、第 1 周

### 📄 线上实习第一周

#### 1️⃣ 机器人基础知识

内容包含：

- 什么是自由度（DoF）

- 关节型机器人 vs 笛卡尔型机器人

- 关节空间 vs 笛卡尔空间（用图说明）

- 常见坐标系：

    - 基坐标系

    - 末端坐标系

    - 工具坐标系

- 三种控制方式：

    - 位置控制

    - 速度控制

    - 力矩控制

---

#### 2️⃣ 灵巧手入门指南

内容包括：

- 什么是灵巧手

- 灵巧手 vs 普通夹爪

- 灵巧手的三大核心差异维度：

    - 结构（自由度、关节分布）

    - 传感器（有 / 没有、类型）

    - 操作算法

---

#### 3️⃣ LinkerHand 官方产品手册

- LinkerHand 产品手册
👉 要求：**重点读 L****6**** / ****O6**** / L2****0**** 的产品****手册**

**学习目标：**

- 区分三款灵巧手：

    - 自由度差异

    - 驱动方式

    - 适合的应用场景

---

### 🎥 学习视频

#### 1️⃣ 机器人坐标系直观理解推荐视频

https://www\.bilibili\.com/video/BV18MameREAE/?spm\_id\_from=333\.337\.search%02card\.all\.click\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336

---

#### 2️⃣ 机器人关节空间及笛卡尔空间推荐学习视频

https://www\.bilibili\.com/video/BV1bt421j7RY/?spm\_id\_from=333\.337\.search%02card\.all\.click\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336



**第一周作业：**

- 结合机器人基础知识学习内容，完成两项任务：① 用文字简述关节型机器人与笛卡尔型机器人的核心区别；② 手绘（或用工具绘制）简单图示，分别展示关节空间与笛卡尔空间的差异（无需复杂建模，清晰表达核心逻辑即可）。

- 阅读 LinkerHand 的 L6/O6/L20 产品手册，整理一份简洁总结表，明确三款灵巧手的自由度、驱动方式及适合的应用场景。

---

## 二、第 2 周

### 主题：URDF 结构 \+ 灵巧手 URDF 解读

---

### 📄  线上实习第二周——URDF 文件核心参数与结构详解

#### 4️⃣ URDF 文件核心参数与结构详解

- URDF 是干什么的

- link 是什么

- joint 是什么

- parent / child 是什么意思

- 这些字段在干嘛：

    - origin

    - axis

    - limit

    - inertial

    - visual

    - collision

---

#### 5️⃣ LinkerHand URDF 阅读指南

内容包括：

- 文件结构怎么分

- 一个手指通常由哪些 joint 组成

- 左右手 URDF 有什么差别

- 哪些 joint 是你在仿真 / RL 中最关心的

---

### 🎥 视频

#### 3️⃣ URDF 坐标系讲解视频

- 灵巧手 URDF 坐标系含义

- 视频链接：https://www\.bilibili\.com/video/BV1Dyf5YLEZa/?spm\_id\_from=333\.788\.videopod\.sections\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336（重点看合集中前六个视频即可）

- 要求学员：

    - 跟着视频暂停看 URDF 文件

    - 理解 LinkerHand URDF的建模思路

---

#### 4️⃣ URDF 可视化讲解视频

内容最好包含：

- URDF 在 RViz / Vscode中怎么看

- joint 转动时模型怎么动

- Vscode：

\[vscode可视化urdf文件教程\.mp4\]

- Rviz：

\[ws\_urdf\.rar\]

跟着压缩包中的md文件和视频操作即可

通过将URDF模型可视化可以更好地理解URDF文件中各部分的作用



**第二周作业：**

- 选取任意一个 URDF 文件（可使用学习资料中的示例文件），找到其中一个`link`和一个`joint`，逐条解释其核心字段（origin、axis、visual等）的具体含义。

- 阅读 LinkerHand URDF L20的左右手URDF代码，回答两个问题：① 一个 LinkerHand 手指通常由哪些关节（joint）组成？② 左右手 URDF 的主要差别体现在哪些方面？

- 按照可视化讲解视频及配套教程，在 RViz 或 Vscode 中成功打开一个 URDF 模型，观察并记录：当任意一个关节（joint）转动时，模型的联动变化规律，同时提交成功可视化URDF的录屏。

---

## 三、第 3–4 周

### 主题：Isaac Sim \+ 强化学习基础

---

### 📄 文档

#### 6️⃣ Isaac Sim 环境安装教程

推荐以下安装教程：

- https://mp\.weixin\.qq\.com/s?\_\_biz=MzkyMTc1NTI3Ng==\&amp;mid=2247487518\&amp;idx=1\&amp;sn=17fb16373e2927e8919eaf0bf6706e2e\&amp;chksm=c0c39b8baedf5f387bbe2a88b33988d0998ad977fa4e431ce864c2c55e0a5b93be48500c2769\&amp;mpshare=1\&amp;scene=23\&amp;srcid=012203J7HxEJK1SFe0OaoEE5\&amp;sharer\_shareinfo=ea5b9c41428fbb4717b2def10a3f5d05\&amp;sharer\_shareinfo\_first=ea5b9c41428fbb4717b2def10a3f5d05\#rd

- 也可以使用课程为大家统一开放的云平台服务器，具体教程参考群通知即可。

- 利用Issac Sim导入URDF模型并成功实现一个简单手势，代码和具体视频教程如下：

\[L6\_left\.zip\]

\[比手势\.mp4\]



---

### 🎥 视频

#### 5️⃣ 强化学习算法讲解视频

首先通过以下视频快速入门强化学习，重点学习PPO算法及TD3算法：https://www\.bilibili\.com/video/BV1Pqj2zaEsP/?spm\_id\_from=333\.337\.search\-card\.all\.click\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336

接下来可通过该视频进一步学习并理解MAPPO算法：

https://www\.bilibili\.com/video/BV1G7o2YTE16/?spm\_id\_from=333\.337\.search\-card\.all\.click\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336



**第三\~四周作业：**

- 提交成功部署Issac Sim环境的截图（云平台和本地部署任一即可），以及成功实现简单手势的录屏或截图。

- 输出一份强化学习算法的学习笔记，内容包括：

1\.简单说明 PPO 算法的核心思路

2\.请判断 PPO 和 TD3 哪种算法更适合处理 “机器人关节连续转动” 这类动作（连续动作空间），并简单说说理由

3\.对比 PPO 和 MAPPO，说说 MAPPO 最核心的优势是什么？它主要是为解决什么类型的任务（单智能体 / 多智能体）而设计的？



---

## 四、补充资料

- 答疑文档：

\[第二周答疑\.docx\]

\[urdf补充mimic\.docx\]

- Isaac sim里面常见的仿真传感器查看及添加方法：

常见的仿真传感器：
左上角 \- Create \- Sensors \-  这里面就列出来了可以添加的传感器：
Generic Sensors —— 通用传感器
Camera and Depth Sensors —— 相机与深度传感器
Contact Sensor —— 接触传感器
Imu Sensor —— IMU传感器（惯性测量单元）
RTX Lidar —— RTX激光雷达
RTX Radar —— RTX雷达
PhysX Lidar —— PhysX激光雷达
LightBeam Sensor —— 光束传感器

添加摄像机：
左上角 \- Create \- Camera \- 拖动调整
查看摄像机视角：左上角 \- Window \- Viewports \- 把 Viewport2 点开，会多出来一个窗口，选择一个，上方 Perspective \- Cameras \- Camera 。点那个有锁的图标就变成摄像机视野了。

视频链接：

https://www\.bilibili\.com/video/BV1XAn6zNEKm/

- 强化学习在灵巧手中的应用：

https://www\.bilibili\.com/video/BV1xyAAeHEjy/?spm\_id\_from=333\.337\.search\-card\.all\.click\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336

https://www\.bilibili\.com/video/BV1Chj9zUEmv/?spm\_id\_from=333\.337\.search\-card\.all\.click\&amp;vd\_source=acca85cb76f4165cc2d59afe3a629336



