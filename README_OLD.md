# Awesome Dexterous Hand & Tactile Manipulation 🖐️🤖

> 汇总 **灵巧手硬件 / 仿真环境 / 数据集 / 算法 / 触觉传感 / 学习路径** 的一站式索引。  
> 用来：复现实验、做课程、写综述、搭教学/展会 Demo，都能直接抄作业。

---

## 目录

- [一、开源硬件手（五指 / 人形 & 可复现）](#一开源硬件手五指--人形--可复现)
- [二、低成本 / 三指平台（研究基线 & 任务平台）](#二低成本--三指平台研究基线--任务平台)
- [三、仿真环境 & 学术基准（手部）](#三仿真环境--学术基准手部)
- [四、数据集（Dexterous Grasp / In-hand / 演示）](#四数据集dexterous-grasp--in-hand--演示)
- [五、算法 / 策略与遥操作（附代码）](#五算法--策略与遥操作附代码)
- [六、触觉硬件 & 软件栈（适合灵巧操作）](#六触觉硬件--软件栈适合灵巧操作)
- [七、学习方案：从基础到 UniDexGrasp + Isaac Lab](#七学习方案从基础到-unidexgrasp--isaac-lab)
  - [M1：基础入门月（第 1–4 周）](#m1基础入门月第-1–4-周)
  - [M2：仿真进阶月（UniDexGrasp + Isaac Lab， 第 5–8 周）](#m2仿真进阶月unidexgrasp--isaac-lab-第-5–8-周)
- [八、如何使用这份清单](#八如何使用这份清单)

---

## 一、开源硬件手（五指 / 人形 & 可复现）

> 主打：**低成本、可复现、带完整文档**，适合科研 + 教学 + 个人复刻。

### 1. LEAP Hand（CMU）

- 定位：低成本、强力耐用五指手，含装配指南、URDF、ROS/Python API、Isaac Gym 仿真、人手到机器人映射。
- 官网：  
  - 官方站点（LEAP 机器人手）：  
    https://leaphand.com  
  - V2 / Advanced 版本站点：  
    https://v2-adv.leaphand.com
- 论文与代码：  
  - RSS 2023 论文（含开源内容）：  
    https://www.roboticsproceedings.org/rss19/p089.pdf  
  - GitHub 组织（API / 仿真 / 双手遥操作等）：  
    https://github.com/leap-hand  

---

### 2. ORCA Hand（ETH Zurich Soft Robotics Lab）

- 定位：17-DoF、腱驱、集成触觉，**一天内可装好**，材料费 ≲ 2k CHF，提供完整设计与控制栈。
- 资源：
  - 官网：https://www.orcahand.com  
  - ETH 项目页：https://srl.ethz.ch/orcahand.html  
  - arXiv 论文（2025）：https://arxiv.org/abs/2504.04259  
  - 控制核心代码：https://github.com/orcahand/orca_core  
- 特点：站点中含 BOM、装配、固件、张力校准脚本。

---

### 3. DexHand（The Robot Studio）

- 定位：低成本开源五指手，完整 3D 打印件与 BOM。
- GitHub 仓库：  
  https://github.com/TheRobotStudio/V1.0-Dexhand  
- 主页：  
  https://www.dexhand.org/

---

### 4. Yale OpenHand Project（耶鲁 GRAB Lab）

- 定位：快速成型 / 低成本开源手族，多款欠驱动设计。
- 资源：
  - 项目页：https://www.eng.yale.edu/grablab/openhand/  
  - 综述论文（RAM 2017）：  
    https://www.eng.yale.edu/grablab/pubs/Ma_RAM2017.pdf  

---

### 5. RBO Hand 2（TU Berlin）

- 定位：充气软体灵巧手，适合安全接触 / 软体抓取研究。
- 资源：
  - CAD 数据集（可打印模具与装配 CAD）：  
    https://depositonce.tu-berlin.de/items/8b6c48a6-dcc1-4466-8eda-cb0df6136b1b  
  - 论文 PDF：  
    https://depositonce.tu-berlin.de/bitstream/11303/6801/1/0278364915592961.pdf  

---

### 6. InMoov i2Hand / 手前臂（Maker 社区）

- 定位：全尺寸可 3D 打印开源人形平台的手部模块。
- 资源：
  - 手 & 前臂模块：https://inmoov.fr/hand-and-forarm/  
  - InMoov 主站：https://inmoov.fr/  

---

## 二、低成本 / 三指平台（研究基线 & 任务平台）

> 为 RL / IL / 控制算法提供 **标准化硬件基线**，经常出现在 benchmark 中。

### 1. TriFinger（ODRI）

- 定位：三指灵巧平台，硬件图纸与软件全开源（BSD-3）。
- 资源：
  - 文档：https://open-dynamic-robot-initiative.github.io/trifinger_docs/  
  - GitHub 组织：https://github.com/open-dynamic-robot-initiative  
  - 论文（PMLR 2021）：  
    https://proceedings.mlr.press/v155/wuthrich21a/wuthrich21a.pdf  

---

### 2. ROBEL D’Claw（Google Research）

- 定位：9-DoF 三指研究平台，带仿真与硬件资料。
- 资源：
  - GitHub：https://github.com/google-research/robel  
  - 平台介绍（D’Claw）：  
    https://sites.google.com/view/roboticsbenchmarks/platforms/dclaw  

---

## 三、仿真环境 & 学术基准（手部）

> 做算法对比 / RL / IL / World Models 时常用的仿真任务。

- **Gymnasium-Robotics · Adroit / ShadowHand 任务族**  
  - Adroit（Door / Hammer / Pen / Relocate）：  
    https://robotics.farama.org/envs/adroit_hand/  
  - ShadowHand 任务（Block / Egg / Pen etc.）：  
    https://robotics.farama.org/envs/shadow_dexterous_hand/  

- **IsaacGymEnvs（NVIDIA）**  
  - 带 ShadowHand 等 dex 任务与 DexPBT 代码：  
    https://github.com/isaac-sim/IsaacGymEnvs  

- **Bi-DexHands（PKU-MARL）**  
  - 双手 / 双臂 dex 任务，高并行 Isaac Gym：  
    https://github.com/PKU-MARL/DexterousHands  

- **ManiSkill Dextrous 子集**  
  - 文档（D’Claw 旋阀等灵巧任务，多难度等级）：  
    https://maniskill.readthedocs.io/en/latest/tasks/dextrous/  
  - 仓库：https://github.com/haosulab/ManiSkill  

- **DexArt（Dexterous Articulated Object Manipulation）**  
  - 基于 Allegro / Shadow 等的可泛化关节物体任务：  
    论文（ar5iv）：https://ar5iv.labs.arxiv.org/html/2305.05706  

- **Safety-Gymnasium · ShadowHand 扩展任务**  
  - 例如双手抛接等带安全约束的灵感任务：  
    https://safety-gymnasium.readthedocs.io/en/latest/environments/safe_isaac_gym/shadowhand_catch_over2_underarm_safe_finger.html  

- **GraspIt!（Columbia）抓取仿真器**  
  - 支持任意手型与接触建模：  
    - 站点：https://graspit-simulator.github.io/  
    - 手册：  
      https://www.cs.columbia.edu/~cmatei/graspit/html-manual/graspit-manual_1.html  

---

## 四、数据集（Dexterous Grasp / In-hand / 演示）

> 做模仿学习 / 抓取生成 / 表征学习的「粮仓」。

- **DexYCB（NVIDIA 等）**  
  - 项目页：https://dex-ycb.github.io/  
  - 工具包：https://github.com/NVlabs/dex-ycb-toolkit  
  - 论文：https://arxiv.org/abs/2104.04631  

- **DexGraspNet & DexGraspNet 2.0（CoRL 2024）**  
  - v1 仓库：https://github.com/PKU-EPIC/DexGraspNet  
  - v2 仓库：https://github.com/PKU-EPIC/DexGraspNet2  
  - 主页：https://pku-epic.github.io/DexGraspNet/  

- **ContactPose（Meta / GT 等）**  
  - 主页：https://contactpose.cc.gatech.edu/  
  - GitHub：https://github.com/facebookresearch/ContactPose  

- **DexCap（RoboConf 2024）**  
  - 可移动手部动捕系统 + DexIL 模仿学习，直连灵巧手：  
    - 项目页：https://dex-cap.github.io/  
    - 论文：https://arxiv.org/html/2403.07788v1  
    - 会议页：https://roboticsconference.org/2024/program/papers/43/  

- **RealDex**  
  - 真实人类风格的灵巧抓取 teleop 轨迹，多视角多模态：  
    https://4dvlab.github.io/RealDex_page/  

- **D4RL · Adroit 离线 RL 数据**（Pen / Relocate / Hammer / Door等）  
  - Pen 数据集：https://minari.farama.org/v0.5.1/datasets/D4RL/pen/  
  - Relocate：https://minari.farama.org/main/datasets/relocate/  
  - 综述（ar5iv）：https://ar5iv.labs.arxiv.org/html/2004.07219  

- **DexGrasp Anything（2025）**  
  - 覆盖 1.5 万+ 物体、340 万+ 抓取姿态：  
    - 主页：https://dexgraspanything.github.io/  
    - 论文：https://arxiv.org/html/2503.08257v1  

---

## 五、算法 / 策略与遥操作（附代码）

> 从「怎么采数据」到「怎么学策略」，再到底层遥操作硬件。

- **DexHandDiff（CVPR 2025）** – 交互感知扩散规划
  - 项目页：https://dexdiffuser.github.io/  
  - 代码：https://github.com/Liang-ZX/DexHandDiff  
  - 论文 PDF：  
    https://openaccess.thecvf.com/content/CVPR2025/papers/Liang_DexHandDiff_Interaction-aware_Diffusion_Planning_for_Adaptive_Dexterous_Manipulation_CVPR_2025_paper.pdf  

- **DexDiffuser（抓取生成）**
  - 论文页：https://yulihn.github.io/DexDiffuser_page/  
  - 代码：https://github.com/YuLiHN/DexDiffuser  

- **UMI（Universal Manipulation Interface）**
  - 主页：https://umi-gripper.github.io/  
  - 论文：https://arxiv.org/html/2402.10329v3  
  - 代码：https://github.com/real-stanford/universal_manipulation_interface  

- **DexMV（从人类视频到灵巧演示/策略）**
  - 主页：https://yzqin.github.io/dexmv/  
  - 仿真 / 重定向：https://github.com/yzqin/dexmv-sim  
  - 论文（ECCV 2022）：  
    https://www.ecva.net/papers/eccv_2022/papers_ECCV/papers/136990562.pdf  

- **Bidex（CMU）双手遥操作采集系统**
  - 主页：https://bidex-teleop.github.io/  
  - LEAP 手套遥操作示例：  
    https://github.com/leap-hand/Bidex_Manus_Teleop  

- **DOGlove（RSS 2025）低成本开源手套**
  - 面向 LEAP 等灵巧手的遥操作与模仿学习数据采集（承诺开源全栈）：  
    - 论文：https://www.roboticsproceedings.org/rss21/p104.pdf  
    - 预印本：https://arxiv.org/html/2502.07730v1  

---

## 六、触觉硬件 & 软件栈（适合灵巧操作）

> 从 DIGIT 到 OmniTact，再到仿真与 ML 库，一套闭环。

- **DIGIT（Meta/FAIR）**
  - 设计文件：https://github.com/facebookresearch/digit-design  
  - 接口库：https://github.com/facebookresearch/digit-interface  

- **Digit 360 & Digit-Plexus**
  - Digit 360：https://github.com/facebookresearch/digit360  
  - Digit-Plexus：https://github.com/facebookresearch/digit-plexus  

- **OmniTact**
  - 主页：https://sites.google.com/berkeley.edu/omnitact  
  - 代码 / 测试平台：https://github.com/s-tian/bench-press  

- **GelSlim 3.0 / 4.0**
  - 仓库：https://github.com/mcubelab/gelslim  
  - 论文：https://arxiv.org/abs/2103.12269  

- **TacTip / DigiTac**
  - DigiTac 设计：https://github.com/nlepora/digitac-design  
  - 介绍：https://lepora.com/digitac/  

- **TACTO（触觉仿真）**
  - PyBullet 版：https://github.com/facebookresearch/tacto  
  - 论文页：https://ai.meta.com/research/publications/tacto-a-fast-flexible-and-open-source-simulator-for-high-resolution-vision-based-tactile-sensors/  
  - MuJoCo 分支：https://github.com/L3S/TACTO-MuJoCo  

- **Tactile-Gym 2.0**
  - GitHub：https://github.com/ac-93/tactile_gym  

- **PyTouch（Meta）**
  - GitHub：https://github.com/facebookresearch/PyTouch  
  - 文档：https://facebookresearch.github.io/PyTouch/  

---

## 七、学习方案：从基础到 UniDexGrasp + Isaac Lab

> 下面是一个 **8 周灵巧手学习 / 实习路径**，直接复用即可。  
> M1：基础入门月（打基本功）  
> M2：仿真进阶月（UniDexGrasp + Isaac Lab + 可交付任务）

---

### M1：基础入门月（第 1–4 周）

#### 第 1 周：机器人基础 & 灵巧手初步了解 + 环境前置准备

**核心主题：**  
机器人学基础概念 + 灵巧手整体认知 + 环境前置配置

**具体学习内容：**

1. 机器人基础知识
   - 自由度（DoF）、关节型机器人 vs 笛卡尔型机器人
   - 关节空间 vs 笛卡尔空间
   - 常见坐标系：基坐标系、末端坐标系、工具坐标系
   - 常见控制方式：位置控制 / 速度控制 / 力矩控制

2. 灵巧手入门资料
   - 入门文章：**《灵巧手入门指南 / 目录与概述》**（可自备或课程统一发放）
   - 灵巧智能 Dexhand 官网产品手册：  
     https://www.dex-robot.com/productionDexhand  
   - 重点理解：
     - 不同自由度、不同驱动方式的灵巧手，在功能上的差异
     - 区分不同灵巧手在 **结构 / 自由度 / 应用场景** 上的不同

**交付物要求：**

1. 学习笔记（≥ 500 字），包括：
   - 灵巧手在 **结构、传感器、操作算法** 三方面的分类
   - 不同类型灵巧手在功能实现上的主要区别
   - 手绘至少一个灵巧手的结构示意（拍照或电子版均可）

---

#### 第 2 周：URDF 基础 & Dexhand URDF 解读

**核心主题：**  
URDF 模型结构 & Dexhand 手部 URDF 解读

**具体学习内容：**

1. URDF 坐标系与基本语法
   - 视频：灵巧手的 URDF 坐标系含义  
     https://www.bilibili.com/video/BV1dwP6eBE87?spm_id_from=333.788.videopod.sections&vd_source=acca85cb76f4165cc2d59afe3a629336
   - 重点理解：
     - `link`、`joint` 的含义
     - `origin`、`axis`、`inertial`、`visual`、`collision` 等字段的作用

2. 下载与阅读 Dexhand URDF
   - 仓库：https://github.com/DexRobot/dexrobot_urdf  
   - 工具：
     - VSCode + URDF 插件
     - RViz 或其他 URDF 可视化工具

**交付物要求：**

- 一份系统的 URDF 学习笔记，须包含：
  - URDF 基本结构与常用字段解释
  - 任选一种灵巧手，逐段分析 URDF 文件含义：
    - 每个 `joint` 对应哪一关节、旋转轴、关节范围
    - 整体拓扑结构图（建议手绘或用简单工具画出关节连接关系）

---

#### 第 3–4 周：Isaac Gym 仿真环境搭建及灵巧手模型加载

**核心主题：**  
Isaac Gym Preview 4 环境搭建 + 加载 Dexhand URDF + 手动控制关节

**具体学习内容：**

1. 参考教程与代码
   - DexRobot 官方 Isaac 仿真仓库：  
     https://github.com/DexRobot/dexrobot_isaac  

2. 在 Isaac Gym 中完成：
   - 搭建并验证 Isaac Gym Preview 4 环境
   - 加载任一 Dexhand 模型
   - 查询并理解关节命名 / 索引
   - 编写简单脚本手动调节关节角度（如张开 / 闭合手指）

**交付物要求：**

1. 录屏（30–60 秒）：
   - 展示在 Isaac Gym 中手动调节任一灵巧手模型各关节角度（如张开 / 闭合、单指弯曲等）

2. 报错解决记录（≥ 3 条），包括：
   - 原始报错信息
   - 排查思路
   - 最终解决方案（含关键命令 / 配置修改）

---

### M2：仿真进阶月（UniDexGrasp + Isaac Lab， 第 5–8 周）

**阶段目标：**

- 基于 Isaac Gym 完成 UnityDexGrasp 项目的基础复现（数据生成 + 策略训练 + 简单评估）
- 搭建至少 1–2 个基于 Isaac Lab 的 Dexhand 灵巧手任务环境（如台面抓取、in-hand 旋转）
- 初步掌握 **模仿学习 / 抓取生成策略** 的基本思想与实现流程

---

#### 第 5–6 周：UniDexGrasp 学习及复现（数据生成 & 离线训练）

**核心主题：**  
UniDexGrasp 代码结构理解 + 环境配置 + 数据集下载与训练

**具体学习内容：**

1. 按照官方 README 完成 dexgrasp_generation 部分的环境与训练：
   - 文档：  
     https://github.com/PKU-EPIC/UniDexGrasp/blob/main/dexgrasp_generation/README.md  

2. 理解关键模块：
   - 数据集结构与读取方式
   - 抓取姿态生成逻辑
   - 评价指标与可视化方法

**交付物要求：**

1. 训练效果截图或录屏：
   - 包含 loss 曲线、抓取结果可视化（如生成抓取姿态渲染）

2. 代码结构学习笔记（≥ 800 字），建议包含：
   - 项目整体结构图（自己画模块图）
   - 各主要模块功能：
     - 数据（dataset / dataloader）
     - 模型（网络结构）
     - 训练（训练脚本 / 配置）
     - 评估（metrics、可视化）

3. 遇到的问题及解决方案（≥ 3 条）

---

#### 第 7–8 周：UniDexGrasp 仿真联调 + Isaac Lab 灵巧手任务设计

**核心主题：**  
将 UniDexGrasp 策略与仿真环境联动，在 Isaac Lab 上搭建可复用的灵巧手任务（可直接打包给客户 / 高校 / 展会使用）

---

##### 1. UniDexGrasp Policy 仿真联调（Isaac Gym）

- 按 README 完成 dexgrasp_policy 部分：
  - 文档：  
    https://github.com/PKU-EPIC/UniDexGrasp/blob/main/dexgrasp_policy/README.md  
- 在 Isaac Gym 中调用训练好的策略：
  - 将策略输出映射到仿真关节控制
  - 统计抓取成功率 / 物体稳定性

**交付物：**

- 仿真运行策略的录屏或截图
- 一份策略调用流程说明文档：
  - 从 observation → policy → action → 关节命令 的数据流说明

---

##### 2. Isaac Lab 灵巧手任务设计（重点，服务硬件销售 / 教学）

**目标：**

在 Isaac Lab 中围绕 Dexhand 灵巧手，设计 1–2 个可标准化交付的任务环境，作为：

- 高校教学实验
- 企业 PoC 验证
- 展会 & 客户拜访 Demo

**推荐任务类型（至少选一，建议有一个稳定 Demo）：**

- **任务 A：台面抓取任务（Tabletop Grasp）**
  - 场景：桌面放置 1–3 个规则物体（方块 / 圆柱等）
  - 目标：灵巧手抓起物体，并在一定时间内不掉落
  - 输出：
    - 任务脚本 / 配置
    - 成功率统计
    - 视频演示（30–60 秒）

- **任务 B：简单 in-hand 旋转（In-hand Reorientation）**
  - 场景：灵巧手已夹持一个物体
  - 目标：绕指定轴旋转设定角度（如 90°），最终姿态误差控制在阈值内
  - 输出：
    - 姿态误差曲线或统计表
    - 演示视频（30–60 秒）

- **任务 C：展会级 Demo 雏形（选做）**
  - 示例流程：抓起小物体 → 旋转 → 放入指定容器
  - 用于后续展会展示升级版 Demo 的原型

**技术步骤参考：**

1. 将 Dexhand URDF 导入 Isaac Lab，并验证：
   - 关节数量与命名是否与实际一致
   - 关节限制、质量、惯量等物理参数是否合理
2. 编写 Isaac Lab Task/Env：
   - 定义 observation space / action space
   - 定义奖励函数 / 成功条件（初期可以非常简单）
3. 可视化与评估：
   - 统计成功率
   - 导出关键指标曲线（如抓取稳定时长、姿态误差等）

**交付物要求：**

1. **UniDexGrasp 部分：**
   - 仿真中运行策略的录屏或截图
   - 策略调用流程说明（面向工程师）

2. **Isaac Lab 灵巧手任务：**
   - 至少 1 个任务环境的：
     - 配置文件 / 任务脚本（建议上传到 GitHub）
     - 任务说明文档（面向 “客户 / 高校老师”）：
       - 任务目标
       - 硬件需求
       - 教学 / 演示价值
   - 30–60 秒仿真演示视频：
     - 可直接用于 Dexhand “仿真 + 教学 + PoC 套餐” 的展示和销售
   - 遇到的问题及解决方案（≥ 3 条）：
     - 尤其是 URDF 导入、碰撞稳定性、仿真发散等问题

---

## 八、如何使用这份清单

- **做硬件复现：**
  - 优先看：LEAP Hand、ORCA Hand、DexHand、RBO Hand 2
- **做算法 / 对比实验：**
  - 仿真：Gymnasium-Robotics Adroit / ShadowHand、IsaacGymEnvs、Bi-DexHands、ManiSkill Dextrous
  - 算法：DexHandDiff、DexDiffuser、D4RL-Adroit
- **做数据驱动策略：**
  - 采集侧：UMI、Bidex、DOGlove、DexCap、RealDex
  - 学习侧：DexGraspNet、DexYCB、ContactPose、DexGraspAnything
- **做触觉融合：**
  - 硬件：DIGIT / OmniTact / GelSlim / DigiTac
  - 仿真：TACTO / Tactile-Gym
  - 学习库：PyTouch
- **做课程 / 实训：**
  - 直接照抄「第 1–8 周学习方案」，再把任务脚本 + 视频打包成教学 / 展会内容。

