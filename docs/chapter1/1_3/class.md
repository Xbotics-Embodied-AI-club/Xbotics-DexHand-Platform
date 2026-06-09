# 灵巧手实训营第三个月

# M3 线下实操月｜基于灵心巧手O6 SDK demo\+sim2real

---

## 主题：O6 SDK 调用与 Demo 使用教程

---

### 1️⃣ 第9周学习内容    理解 O6 SDK 的调用

第9周 FAQ：[第 9 周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/YAB2w5Yxgif1XvkxIwzcVUqJnlg?from=from_copylink)

关于O6 SDK的解析及讲解视频可参照该文档：[Linkerhand python sdk](https://xcn8mk42amxy.feishu.cn/wiki/PRSpwi6gwiTdJNkqNimcETWJnQc)

1. Linkerbot Python SDK 

    文档：https://docs\.linkerhub\.work/sdk/zh\-cn/guide/installation\.html

    github：https://github\.com/linker\-bot/linkerbot\-python\-sdk



2. LinkerHand C\+\+ API

文档：https://github\.com/linker\-bot/linkerhand\-cpp\-sdk/blob/main/docs/API\-Reference\.md

github:https://github\.com/linker\-bot/linkerhand\-cpp\-sdk



3. LinkerHand ROS2 SDK

文档：https://document\.linkeros\.cn/developer/68

Github: https://github\.com/linker\-bot/linkerhand\-ros2\-sdk

#### SDK 简述

O6 灵巧手有 6 个控制自由度：

|**索引**|**关节名**|**控制内容**|
|---|---|---|
|0|thumb\_flex|拇指弯曲|
|1|thumb\_abd|拇指侧摆|
|2|index|食指弯曲|
|3|middle|中指弯曲|
|4|ring|无名指弯曲|
|5|pinky|小指弯曲|

所有角度值范围为 **0 \~ 100**：

- 0 = 手指完全伸展（张开）

- 100 = 手指完全弯曲（闭合）

示例：

```Plain Text
[100, 100, 100, 100, 100, 100]  → 张开五指
[12,  42,  12,  12,  12,  12]  → 握拳
[100,  80,  12,  12,  12,  12]  → 点赞（拇指100，食指80，其余闭合）
```

O6 可实时读取以下状态：

|**状态字段**|**内容**|**用途**|
|---|---|---|
|angles|6 个关节当前角度|判断手指位置|
|speeds|6 个关节当前速度|运动监控|
|accelerations|6 个关节当前加速度|运动监控|
|torques|6 个关节当前扭矩|碰撞检测、负载判断|
|temperatures|6 个关节温度|过热保护（≥70°C 警告）|
|faults|故障码|异常检测|
|force\_sensor|力传感器矩阵（另选）|抓取力度感知|

### **第九周完成上述SDK内容的理解学习并提交学习周报，第十周开始分批次到各个城市的负责人处进行线下实训，线下实训的内容目前主要有以下几个部分：**

### 2️⃣利用OpenClaw控制灵心巧手O6执行动作

1. 自行尝试“养龙虾”调用SDK控制实机

2. Windows调用已写好的基于官方Python SDK封装O6桥接程序，使Openclaw更便捷地调用SDK

源代码：https://github\.com/fanfan142/Xbotics\-O6

也可直接下载打包程序：

\[Xbotics\_O6控制台\.zip\]

\[o6\_service\.py\]

#### 随后依次按以下步骤操作

**1****\.安装openclaw**

**1****\.1 安装node\.js**

建议24\+，至少22\+：https://nodejs\.org/zh\-cn/download/current

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDhlNTM2OTNlYjc0MGE2OTdhMzg0OGJmYWIzOWFlNTNfODY5OWEwNzhjZjBhNTQ5MTgyMmExODI0MzQ4ZjU2ZTZfSUQ6NzYyMzA3NTcwNjA5NjA5NDQyNV8xNzgwMjIxNTg5OjE3ODAzMDc5ODlfVjM)

下载windows安装程序并安装



**1****\.2 安装openclaw**

1\.检查安装版本：

win\+R 输入powershell，并输入命令

node \-v

npm \-v

2\.安装openclaw:

建议带加速源的汉化版openclaw

npm i \-g @qingchencloud/openclaw\-zh \-\-registry=https://registry\.npmmirror\.com

\(或社区原版https://github\.com/openclaw/openclaw\)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWEzOGE1YzZjZTI5NDgwYTMxYWYwMzg3MTdkMzc4MjlfYzk2YjY0MWZhMTk2YmJiYmRjMGQ0ZGE2MTg2YTY1MGVfSUQ6NzYyMzA3NTkyNzY4NTIzNzk4NV8xNzgwMjIxNTg5OjE3ODAzMDc5ODlfVjM)

3\.初始化: openclaw onboard \-\-install\-daemon

并根据提示配置

4\.启动网关：openclaw gateway

5\.打开控制台访问 127\.0\.0\.1:18789

6\.激活会话发送信息 验证配置完成

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTc0ZjEwOTVhNzEzMjBiNTlmM2ZmYTBhZDFmYjg3YzRfYjQyYzQ0YjhjMGQyOTliM2E0YTNmMmE2MjVjNTM2OTFfSUQ6NzYyMzA3NjE1MzE0NTc0MDIzMF8xNzgwMjIxNTg5OjE3ODAzMDc5ODlfVjM)



**2\.****配置O6环境**

2\.1 安装can驱动：https://document\.linkeros\.cn/developer/72

2\.2 打开“Xbotics\_O6控制台\.exe”应用，将“OpenClaw调用说明”内容全部复制发送给openclaw

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTRhNzViY2YzZWNhNTU0OGI5YWNhYTQ1OTc4ZjNiNGZfYjRhZjBiZDZkYTc0MWVjYzAyMjMwOGE4ZjMxOWQ2NjZfSUQ6NzYyMzA3NzEyMDQ0MDMwNjYzOV8xNzgwMjIxNTg5OjE3ODAzMDc5ODlfVjM)



**3****\. 让openclaw测试比个耶吧\~**

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDhjNmU2NmU4NGFkNTg3NjNjODBjYWNhOGI3M2U4NjdfMThmMDg0MDJmMDY1MGMxOTdhNGU1NDNhM2Y3ZjY0NjVfSUQ6NzYyNDU0OTA5NDQzMzg2ODk4Nl8xNzgwMjIxNTg5OjE3ODAzMDc5ODlfVjM)

### 3️⃣ Xbotics\_O6控制台 实机测试demo

1. 在控制台程序里，观察左下角O6连接状态

2. 点击左侧常规手势，观察实机是否执行运动

3. 摄像头跟随demo：选择摄像头设备号，启动摄像头，启动跟随

（注意左右手与实机对应，初次运行请标定）

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjdlMGUzYzNmMTFmYTQwMjlkZmViYzk4ZDgxYTJkYzBfMDMzYmRmZTUxN2I2OTc5ZjA3OTc4N2M3ZjdiNDJkYjVfSUQ6NzYyMzA4MDk2NDQzMTgwOTc1OF8xNzgwMjIxNTg5OjE3ODAzMDc5ODlfVjM)

4. 猜拳demo：做出“石头\-剪刀\-布”手势，点击开始猜拳

演示视频：

\[控制台演示\.mp4\]

### 第三个月线下实训\&amp;结营要求：

1. 在北京/上海/深圳/西安/武汉及这五个城市附近愿意参加线下实训的同学自行联系对应城市的负责人；可以定在周末多个人一起线下实训，实训费用为500/天，多个人可共同分担实训费用；能力较强且有强烈意愿参与灵巧手真机开发的同学可以申请与所在地区的负责人在之后共同开发更多的灵巧手真机项目，共同开发期间免除费用且由社区负担开发所消耗的API token费用。

2. 拍摄成功实现真机demo的视频并分享到实训营种子群里或小红书等社交平台。

3. **完成前两个月的线上实习任务及第三个月线下实训内容的学员可获得实习证明；确实没有条件或因为地理位置原因不便参加线下实训的学员完成下列Isaacsim仿真任务之一并提交成果代码及视频可获得实习证明：**

    1\. 将第二个月的linkerhand\-unidexgrasp项目的policy部分移植到Isaacsim上面并优化PPO策略及奖励函数实现成功抓取（可以基于文档[第8周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/WK57wqw2biPVCDky01tcXfrRntd?from=from_copylink)里已经初步移植的部分进行进一步优化，也可以自己借助agent插件进行移植及优化）

    2\. 在Isaa csim里面实现一个机械臂\+灵巧手（灵巧手模型必须为L6/O6/L20其中的一种，机械臂模型随意）的强化学习抓取训练，要求以灵巧手在仿真环境中成功抓握物体为准（抓握任意一个物体即可），可以复现已有的相关开源项目并将灵巧手模型修改为L6/O6/L20其中的一种。

    3\. 在Isaacsim里面实现一个其它的灵巧手抓取仿真项目，灵巧手模型必须为L6/O6/L20其中的一种，可以复现已有的相关开源项目并将灵巧手模型修改即可。










