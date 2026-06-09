# 灵巧手线上实习第二个月 

# M2 **仿真进阶月｜UniDexGrasp项目\+ Isaac sim 灵巧手动作可视化**

---

### 主题：灵巧手Demo示例 \+ 项目复现 

---

## 一、第 5 周

本周问题讨论区，欢迎大家把解决方法放上去： [第5周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/H0iQwqlQ8izF0EkYDJtc8tNNnkh?from=from_copylink)

#### 1️⃣ 用Issac Lab跑通灵巧手强化学习示例demo

**学习目标：**

- 用Issac Lab成功跑通一个灵巧手抓取小球的强化学习demo

关于Issac Lab的环境配置，在第五周的FAQ文档里有详细教程

代码及视频教程：

\[handrl\_grasp\.zip\]

\[灵巧手抓握小球视频教程\.mp4\]

- 用Issac Lab成功跑通一个灵巧手关节到达指定位置的强化学习demo

代码及效果视频：

\[handrl\.zip\]

\[灵巧手关节到达指定位置效果视频\.mp4\]

- 分析代码中的PPO算法

#### 2️⃣ Issac Gym安装

**完成内容：**

- Isaac Gym仿真环境搭建及仿真模型操控

- 按照该链接教程 [https://github\.com/linker\-bot/linkerhand\-sim/tree/main/linker\_hand\_isaac\_gym\_urdf](https://github.com/linker-bot/linkerhand-sim/tree/main/linker_hand_isaac_gym_urdf) 完成Issac Gym环境配置（版本使用Isaac Gym Preview 4）

- 云平台中配备有现成的Issac Gym，也可以直接使用云平台中的Issac Gym进行后续仿真



**第五周作业：**

- 提交成功跑通两个demo的录屏或截图。

- 这两个demo中的PPO算法中奖励函数的构建思路。

- Issac Gym环境成功部署截图（本地或云平台均可）。

---

## 二、第 6 周（本周内容不用跑代码，只用了解）

本周问题讨论区，欢迎大家把解决方法放上去：[第6周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/DQB1wAWSIiS9PGkJN1xcwg2GnUK?from=from_copylink)

#### 3️⃣ Dexgraspnet项目结构解析

- 仔细阅读论文，理解项目结构

论文链接：https://arxiv\.org/abs/2210\.02697

\[DexGraspNet\-main\.zip\]

- 重点理解：

1\. Dexgraspnet 的作用是什么

2\. grasp\_generation（抓取生成模块）的目录结构与各部分作用

3\. Dexgraspnet 与 Unidexgrasp项目的联系

---

#### 4️⃣ Linkerhand\-unidexgrasp项目结构详细解析

\[Unidexgrasp原项目论文\.pdf\]

\[Linker\_UniDexGrasp 项目结构解读\.docx\]

\[linkerhand\-init\.zip\]

\[dexgrasp结构讲解1\.mp4\]

\[dexgrasp结构讲解2\.mp4\]

- 仔细阅读解读文档及压缩包里面每个文件夹目录下的txt文件，理解该项目的实现思路

- 重点理解 linkerhand\-init\\linkerhand\-unidexgrasp\-main\\dexgrasp\_policy\_l20hand\\dexgrasp\\algorithms\\rl\\ppo 目录下module\.py ppo\.py storage\.py 这三个代码文件所构建的PPO算法的整体构建思路及核心流程；

\[PPO 强化学习代码详解：从基础概念到核心模块\.docx\]

- PPO算法核心流程图链接：https://p11\-flow\-imagex\-sign\.byteimg\.com/tos\-cn\-i\-a9rns2rl98/cfa9e30533304f6091e2add51f2d6506\.png\~tplv\-a9rns2rl98\-image\.png?lk3s=8e244e95\&amp;rcl=20260319001218602D9F5CD3C94EDFDCA5\&amp;rrcfp=dafada99\&amp;x\-expires=2090074338\&amp;x\-signature=AkeWqWQwbFIkKEVfPvAWarOMz3Q%3D

- 能力较强的同学可以进一步学习 \~\\algorithms\\rl 目录下其他算法的构建思路及核心流程，这里不作强制要求。

- 每位学员任选该项目中的一个模块进行深入解读

- **训练营第一期的优秀学员“4001ish”对这两个项目做了非常详细且深入的解析，想深入研究这两个项目的同学也可以进一步学习，讲解视频链接：**https://meeting\.tencent\.com/crm/2kOqjBoo4a

- **解析文档：**

\[DexGraspNet项目分析\.docx\]

\[dexgraspnet项目复现\.docx\]

\[unidexgrasp项目分析\.docx\]

\[unidexgrasp项目复现\.docx\]

**第六周作业：**

- 分别提交关于两个项目整体结构及关于两个项目中某一模块深入解读的学习笔记。

- 提交关于Unidexgrasp中的PPO算法整体构建思路及核心流程的学习笔记。

- Unidexgrasp项目相比于DexGraspNet1\.0项目有哪些优势，二者之间的联系体现在哪些方面。

---

## 三、第 7–8 周

本周问题讨论区，欢迎大家把解决方法放上去：[第 7 周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/Sx0jwAhp4iAEd7kowuHclZTHnAh?from=from_copylink)   \&amp;   [第 8 周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/WK57wqw2biPVCDky01tcXfrRntd?from=from_copylink)

### 5️⃣ Issac Sim跑通三个简单的灵巧手抓取demo（文档及效果视频）

\[linkerhand\-init\-recurrence\.rar\]

\[动作1\.mp4\]

\[动作2\.mp4\]

\[动作3\.mp4\]

内容包括：

- 阅读 linkerhand\-init\-recurrence\\linkerhand\-init\\linkerhand\-unidexgrasp\-main\\docs 文件夹中的“Isaac\_Sim\_脚本使用说明”和“Isaac\_Sim\_脚本移植说明”两个文档（文件结构已配好），实现目标为“L20手抓取杯子”，“L20手抓取点云”，“shadow手抓点云”三个场景及动作的可视化；若Issac Sim版本不为4\.5\.0（如版本为5\.0\.0或5\.1\.0），则参照同一目录下的“根目录脚本迁移至IsaacSim5\.0\.0说明”进行迁移并跑通可视化。

- 阅读同一目录下的“isaac\_sim\_basic\_env\_L20\_教程”，理解这三个场景及动作的实现思路与参数设置。

---

[第 8 周 FAQ](https://ivjep4wm9di.feishu.cn/wiki/WK57wqw2biPVCDky01tcXfrRntd?from=from_copylink)

### 6️⃣ Isaac Gym 复现 Linkerhand\-unidexgrasp 项目

- 分别根据项目（链接：https://github\.com/linker\-bot/linkerhand\-unidexgrasp）中两个文件夹目录下的readme文件进行该项目的复现

### 需要的环境配置教程：

- **注意：该项目的两个部分，即generation部分和policy部分需要分别创建两个不同的虚拟环境，然后再分别按照两个部分的官方readme文档进行配置！**

- 关于generation部分复现可以参考这个帖子https://zhuanlan\.zhihu\.com/p/650320613，关于在云平台中进行generation部分数据集下载及文件路径报错的问题可查看如下文档**（必看）**

\[云平台中数据集下载方法及文件路径报错解决\.docx\]

- 在云平台上按照readme文件配置policy部分的环境时常见的报错及解决方法已整理在文档中**（配置policy部分时按照官方md一步步配置即可，出现问题或报错时优先使用AI工具解决，如果问题或报错刚好与文档中的问题和报错重合，那么可以使用文档中的解决办法，千万不要照着此文档配置环境！！！）**同学们遇到其它问题及报错的解决方法可以放在FAQ文件中分享给大家

\[报错解决方案\.docx\]

- 分别跑通Linkerhand\-unidexgrasp 项目的generation部分和policy部分。**注意：policy部分只需跑通PPO相关的训练即可，因为跑通其它算法训练所需的部分参数及依赖与PPO所需参数及依赖相矛盾，所以本次实训营policy部分只需跑通PPO相关训练**

- 由于该项目的两个部分（generation和policy）对GPU有较高要求且运行时间都比较久，所以时间不充裕的学员可先提交成功运行训练的录屏或截图**（policy部分只需运行成功关于PPO算法的仿真训练即可）**



**第七\~八周作业：**

- 提交成功在Issac Sim里面可视化三个灵巧手抓取场景的截图（云平台和本地部署任一即可）。

- 提交关于这三个场景及动作的实现思路与参数设置的学习笔记。

- 提交成功运行Linkerhand\-unidexgrasp项目以及修改某些参数之后再次成功运行该项目的截图（云平台和本地部署任一即可，需分别成功运行项目的两个部分——generation部分及policy部分），如果有成功可视化训练结果的录屏或截图更好。若有其他报错可提交报错内容\+解决办法。





## 挑战任务（学有余力的同学可尝试完成，不作强制要求）：

- 随机化代码解读参考：

\[random\.md\]



---

