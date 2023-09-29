# Auto Focus Assistant

## 基于目标检测和立体视觉的辅助对焦系统<br>

#### this is a system using a depth camera to detect objects and control a follow focus motor, aiming to drive the focus ring and achieve auto focus.<br>

### **preface/前言**

##### 本项目基于GPL3.0协议开源，依据协议要求，后续基于此代码的项目需保持GPL3.0开源<br>
##### 代码主要内容包括：
- 英特尔RealSense相机数据读取和参数设置
- 铁头原力N马达和手柄通信协议解析
- 多种目标检测网络部署
- 简易追踪逻辑和模式切换
- 便于拓展的代码框架搭建
- 基于拉格朗日插值的对焦曲线计算
- 支持多镜头(默认5支，可自行按需添加)的校准和数据写入
- 简易对焦UI绘制<br>
##### 希望能以我微薄的力量，推动影视工业化和智能化发展<br>

### **project introduction/项目介绍**
#### 英文版项目介绍，截取自个人作品集（对比部分可能不严谨哈各位轻喷谢谢）
![5](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/b766791b-7a17-43dc-b47d-e891f787f9be)
![6](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/b9f0e13f-10d6-4984-8c4b-ce46924f2e43)


### **background/研究背景**
近年来，随着个人Vlog和短视频的兴起，视频拍摄的门槛越来越低，用户对于摄影机性能的要求也越来越高<br>
  对于个人用户或小团队用户而言，摄影机性能主要体现在两个方向: **画质和易用性**<br>
  前者在此处不作详细展开，后者主要包括：体积重量、人体工学、对焦性能和防抖性能等。<br>
  其中， **自动对焦系统** 是极为重要的一部分，而各厂家由于技术路线或专利的限制，对焦性能参差不齐<br>
  对于canon和sony的摄影机，他们往往有着丰富的对焦选择和稳定的对焦性能，得到了大量用户的认可<br>
  而对于来自Panasonic和blackmagic design的摄影机，他们有着出色的画质但缺乏可靠的自动对焦系统<br>
  在当下的影像市场上，有很多画质优秀的影像设备由于易用性不佳，而导致了大量用户的流失<br>
* #### 本项目旨在改善摄影机的自动对焦性能，基于目标检测和追踪，实现自动或半自动对焦

### **principle/主要原理**
- 本项目属于主动式对焦系统<br>
  基于结构光立体视觉技术，通过深度相机获取深度图和彩色图，并将二者进行匹配<br>
  通过计算机视觉技术检测并追踪图像中的目标，包括人脸、人体和其他物体，并实时读取目标的距离<br>
  通过距离-电机位置映射函数，驱动电机带动对焦环至目标位置，完成对焦动作<br>

### **hardware structure/硬件系统结构**
+ #### range finder/测距仪<br>
    + 用于测量目标与相机的距离，常用的测距仪包括深度相机和各类激光雷达
    + 本项目所使用的测距仪是两款深度相机：奥比中光AstraPro 和 英特尔Realsense D415(兼容435/455/457，未来可能部分兼容D430)
    + 前者的原理的单目红外结构光测距，在室内表现优异，有效距离8m；后者为双目结构光，在室内外均可使用，理论有效距离>10m
    + 由于个人精力有限，大部分工作将基于Realsense相机开发，不能保证对AstraPro提供持续的兼容（V2.0版本已经停止对AstraPro的支持）
    + RealSense D415 如图所示：![image](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/800abcc2-f151-491b-81c0-6c3e389665b5)

+ #### processor/处理器<br>
    + 用于处理从测距仪得到的数据，常用的视觉处理平台有英伟达的NX系列、树梅派等平台和以NUC为代表的小型电脑
    + 综合考虑成本、体积重量和集成化，本方案所采用的处理器之一为英特尔的Computer Stick(电脑棒)，它采用X86架构，能够方便地进行程序开发和运行
    + 另一套方案是本项目当前采取的方案：即采用分布式方案，将算力平台和相机分离，以牺牲集成度的方式大幅降低了成本和重量，并且不需要妥协性能
    + 本项目(V2.0)版本测试平台为Yoga14s笔记本(4800u)，强制默频低功耗运行以降低发热；建议实际使用中替换为性能相近的NUC或迷你主机
    + 已经有老哥将本项目移植至树莓派运行；个人认为最优的解决方案是使用RK3588等廉价且有高算力的平台作为处理器
    + 4800u处理器如图所示：![image](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/99db38ab-26cb-42ad-b7b2-6816e6bf99ef)

+ #### actuator/执行器<br>
    + 执行器是直接驱动镜头进行对焦的组件，一般分为内置的对焦马达和外置的跟焦电机
    + V1.0版本所使用的执行器为外置42闭环步进电机，后续可能会更换为28步进电机或无刷电机(V2.0版本中更新为铁头公司的原力N跟焦马达)
    + 同时，基于ESP32进行蓝牙连接部分相机（如BMPCC和ZCAM）的研发也在进行，便于驱动内置马达的自动镜头
    + 铁头原力N马达如图所示：![image](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/2b42a9e3-318f-497f-9ff1-7927423a09d5)

+ #### controller/控制器<br>
    + 控制器用于实现“手自一体”的自动对焦模式选择和功能切换<br>
    + Tilta铁头公司是一家专业制作相机配件的器材供应商，他们于2018年底推出了原力N(NucleusN)电动跟焦系统<br>
      这套系统在这几年间不断更新壮大，提供了跟焦马达、跟焦手柄和蓝牙跟焦手轮等一系列解决方案<br>
      本项目也采用了铁头公司的原力N系统，主要针对其跟焦马达和跟焦手柄进行研发<br>
    + 在本项目中，通过逆向功能还原其通信协议，原力N系统的跟焦手柄被改造为控制手柄<br>
      本项目对按键和滚轮等功能进行了重映射，具体的使用方法见后文“控制逻辑”部分<br>
      除此之外，该手柄还承担给执行器电机供电的功能<br>
    + 铁头原力N手柄如图所示：![image](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/12c825e1-4ce6-4bb3-9c1a-cd6a5a877f6f)
    + 控制器接线如图所示（单片机端换成USB转串口模块即可）：![image](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/8b207e3b-c33f-4314-b58f-fe4231780c8e)

+ #### monitor/监视器<br>
    + 监视器是监控对焦模式和状态、展示对焦波形图的关键部件
    + 本项目从v2.0版本开始加入了监视器，兼容任意摄影监视器(有hdmi输入即可)
    + 本项目目前采用opencv自带的窗口显示函数进行画面输出，暂未搭建完善的图形界面，因此也不支持触摸操控
    + 示意UI如下（仅为测试Demo，用于分析和排查bug，很丑很混乱，但**又不是不能用**）：![2023-05-28 11-09-34 的屏幕截图](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/b78c331e-2f34-4804-ba9d-42677bfb7ade)
    + 摄影机实际画面如下：![still-1_1 6 1](https://github.com/alexlai2860/Auto-focus-assistant/assets/71208694/85e95b07-0978-4f62-b2aa-433bb5753610)

    + 示意UI说明：
        + 左下角19指的是实时对焦帧率，数值一般在20-25fps(4800u低功耗模式)，取决于处理器性能设定，此处略低是因为开了录屏
        + 左上角的 target-pos 指电机位置（0-9999,除以10即为电机显示屏上的数字）
        + pose 是原力N手柄编码器（滚轮）的值 2570指的是现在手柄指示位置在2.57m tracking指正在追踪目标
            + 注意，如果需要切换追踪的目标（比如切换到远处的一个人），可以转动手柄滚轮，将pose数值调大(远距离)或调小(近距离)，此时系统将解除追踪锁定，会对焦至距离pose指示值最近的目标上
        + 中间上方 drop-count 指的是当前目标丢失帧数(即从当前目标无法检测到开始计算帧数)，默认设定大于8会切换目标，短暂的丢帧不会导致抽搐
        + case-1 指的是模式1，忘记是啥了，貌似是点对焦和区域对焦的切换？请自行测试，应该不用管
        + com:-5 指的是串口通信指令编号，用于测试通信，也不用管
        + AF AI-MODE 指的是系统工作在自动对焦模式下，启用AI目标检测，自动检测人脸进行对焦；这是最常用的模式，但也是抽搐风险最高的模式
        + 此外，系统还提供了MF模式和AF CENTER-MODE，MF下可以使用手柄滚轮进行手动对焦，用波形图辅助；CENTER-MODE则是对焦ROI区域中最近的物体(可在参数文件中修改ROI大小)
        + 中间灰色的框是AI识别框，当面部在框内时可以识别并对焦，框的大小同样可以在参数文件中调整
        + 红色框是人体识别框，1575是人物对焦距离(mm)，优先选择人物眼部距离进行对焦
        + 监视器画面右侧展示对焦波形图，波形图中用红线展示展示手柄滚轮编码器位置，用绿线展示电机实时对焦位置，曲线则为深度映射图(俯视图)

### **software structure/代码文件结构**

#### up/上层<br>
+ #### main<br>
    + main函数：程序入口
+ #### controller/控制器<br>
    + 控制程序运行,负责切换校准模式和对焦模式
+ #### SubController/子控制器<br>
    + 以下两个控制器的基类，未来可能兼容更多的模式
+ #### CalController/校准模式控制器<br>
    + 在校准模式下，调用控制各个modules
+ #### FocusController/对焦模式控制器<br>
    + 在对焦模式下，调用控制各个modules

#### core/核心层<br>
+ #### modules/核心模块:分为以下子模块<br>
+ #### calibrator/校准器<br>
    + 负责对镜头进行校准，建立查找表并写入lens_param.yml
+ #### detector/识别器<br>
    + 负责识别目标
        + FaceDetector:基于卷积神经网络(Yunet)的面部识别，已部署
        + FaceDetectorLight:基于SCRFD的面部识别，已部署
        + ObjectDetector:基于YOLOv5s的目标检测网络，已部署但无法实时检测，暂时搁置
        + ObjectDetectorLight:基于YOLO_fastestV2的目标检测网络，已部署
+ #### dis/距离获取器<br>
    + 负责距离的解算和滤波
+ #### motor/电机驱动器<br>
    + 负责与电机间通信
        + 第一代AF_assistant:基于42闭环步进电机
        + 第二代AF_assistant:基于铁头NucleusN电机
+ #### reader/读取器<br>
    + 负责从深度相机读取彩色视频流和深度信息流
        + 第一代AF_assistant:基于奥比中光相机
        + 第二代AF_assistant:基于Realsense相机

#### bottom/底层<br>
+ #### serial/通信器<br>
    + 用于和电机/相机/单片机等进行串口通信，目前仅支持电机通信
    + Data类为总协议，分为以下子通信协议：
        + TransferData: 程序内部传递数据汇总
        + SendData1/2/3/4/5/6: 程序向电机发送的指令
            + 1~4分别对应步进电机的触发指令、读取指令、修改指令、运动指令
            + 5~6分别对应NucleusN电机的运动指令、校准指令
        + Read: 
            + 对于步进电机:程序从步进电机读取当前脉冲值
            + 对于NucleusN:程序从手柄读取数据
+ #### param/参数文件<br>
    + 用于注入参数，目前分为以下两类参数：
        + param：普通参数，为定值，运行前可以调整
        + lens_param: 镜头标定参数，储存各个镜头的查找表数据，运行时可能改变（如校准/新建镜头）

### **user instruction/使用方法**
#### install dependent lib/安装依赖库<br>
+ OpenCV 4.5.0或更新版本(推荐4.5.4及以上)
+ Realsense SDK 2.0

#### run/运行程序<br>
+ 编译代码
(在项目文件夹下)<br>
```
mkdir build
cd build
cmake ..
make
```

+ 执行程序
若编译成功，则可以进一步执行程序<br>
```
./AF_assistant
```
运行程序后，会提示是否找到原力N<br>
如果出现以下提示:<br>
```
找不到原力N
```
请检查接线，并重新启动程序<br>
若能够找到原力N，则可以进一步选择是否执行自动校准(换镜头后必须校准，其他随意)<br>
```
即将执行自动校准,输入1确认,输入2跳过:
```
(理论上可以支持无限位镜头手动校准，后续有空就加)<br>
校准完成或跳过后，可以选择镜头型号（1～5）,如果需要新增镜头，则按0<br>
```
请输入镜头编号(1/2/3/4/5) 若新建镜头请按0 回车确认
```
以直接执行对焦功能为例，输入镜头编号(如2),回车即系统开始运行<br>
**注意** 如果需要开机自启动，可以在.yml文件中设置好上述参数，即可直接运行<br>

#### auto start/设置开机自启动<br>
由于本项目需要用到imshow，所以开机自启动的设置会略为复杂，也希望有大佬提出改进建议<br>
若采用传统的systemd去启动程序，由于imshow的存在会报错<br>
故本项目自启动思路如下:将项目做成快捷方式，再用系统自带的开机自启动去启动快捷方式<br>
操作步骤如下<br>
+ 在 /usr/local/下创建一个文件夹，名为af-assistant
+ 进入该文件夹，在文件夹内部创建两个.sh文件，分别命名为af-start.sh和gtk-launch.sh(权限问题不再赘述)
+ 前者用于进入项目文件夹并启动项目代码，后者用于进入快捷方式文件夹并启动快捷方式
+ 前者内容示例如下**(可根据代码位置修改)**
```
#!/bin/bash

cd /home/alex/AF_assistant/auto-focus-assistant/build
./AF_assistant
```
+ 后者内容示例如下
```
#!/bin/bash

cd /usr/share/applications/
gtk-launch af-start.desktop
```
+ 随后进入/usr/share/applications/,在该文件夹下创建文件，名为 af-start.desktop
+ 内容如下:
```
[Desktop Entry]
Version=2.0
Type=Application
Name=AF_assistant
Exec=/usr/local/af-assistant/af_start.sh
Terminal=true
Comment=execute af_start.sh
```
+ 打开文件夹，把这个文件拖到桌面文件夹下，获得桌面图标
+ 将桌面图标设置为可执行文件（右键，允许运行）
+ 最后在终端输入 gnome-session-properties
+ 点击添加add，名称可以随意填写，命令填写gtk-launch.sh的地址(/usr/local/af-assistant/gtk-launch.sh)
+ 注意排查名称是否对应正确

+ 串口相关问题：
+ 驱动问题或异常占用：https://blog.csdn.net/a1058191679/article/details/129111054
+ 权限问题：https://blog.csdn.net/Android_WPF/article/details/120892617

#### power limit/功率限制<br>
操作步骤如下<br>
+ 在/usr/local/下创建一个文件夹，名为powerlimit
+ 进入该文件夹，创建名为powersave-mode的.sh文件
+ 文件内容如下(需要对应具体线程数量进行修改!)(以下以8c16t的4800U处理器为例):
```
#!/bin/sh

cpufreq-set -g powersave -c 0
cpufreq-set -g powersave -c 1
cpufreq-set -g powersave -c 2
cpufreq-set -g powersave -c 3
cpufreq-set -g powersave -c 4
cpufreq-set -g powersave -c 5
cpufreq-set -g powersave -c 6
cpufreq-set -g powersave -c 7
cpufreq-set -g powersave -c 8
cpufreq-set -g powersave -c 9
cpufreq-set -g powersave -c 10
cpufreq-set -g powersave -c 11
cpufreq-set -g powersave -c 12
cpufreq-set -g powersave -c 13
cpufreq-set -g powersave -c 14
cpufreq-set -g powersave -c 15

exit 0
```
自启动方面:<br>
+ 方便起见，可以直接在上述的af-start.desktop中进行修改
+ 在文件开头加入以下两行内容即可（修改为用户密码）
```
cd /usr/local/powerlimit/
echo "你的密码"|sudo -S ./powersave-mode.sh
```

#### control logic/控制逻辑<br>
+ NucleusN手柄控制逻辑 & 决策器控制逻辑
按键重映射逻辑如下:<br>
+ 开机和关机
单击开机键开机，长按开机键关机(未改变原始设定)<br>
+ 总模式切换
总模式切换可以在开机后单按REC键(同开机键)实现，同时手柄指示灯颜色改变<br>
总体分为两大模式：自动对焦(绿灯，AF)和手动对焦(红灯，MF)<br>
注意，当模式由自动对焦切换至手动对焦时，默认为锁定状态(很重要，用于规避临时障碍物)<br>
只有手动转动滚轮，才能进入全手动模式<br>
+ 自动对焦
    + 滚轮:用于目标切换<br>
      滚轮用于实时选择不同的目标<br>
      选择逻辑如下:<br>
      滚轮对应编码器数值0-9999，通过非线性重映射到实际距离<br>
      通过滚动滚轮可以切换远近目标<br>
      当滚轮放在最左端，默认对焦最近的目标，且不进行追踪<br>
      当滚轮放在最右端，默认对焦至背景(暂时未实现)<br>
    + CAL按键:用于自动对焦下模式选择<br>
      CAL按键可以切换自动对焦模式，按下按键用于循环切换两种模式<br>
      基于神经网络的人体&人脸混合智能对焦<br>
      中心点对焦/中心区域对焦<br>
      未来将开放更多的物体识别模式<br>
+ 手动对焦
    + 滚轮:用途和直接驱动原力N电机一致<br>
    + CAL按键:用于默认模式和0-10m重映射模式切换<br>
      默认模式即将编码器0-9999线性映射至电机读数的0-999<br>
      0-10m重映射模式即缩小编码器的映射范围至0-10m，搭配示波器使用最佳，手感更好<br>
      另外：默认模式（原始设定）可能有BUG（因为还没有写逆拉格朗日插值的函数），所以建议使用0-10m重映射<br>

+ 附加功能:<br>
    + 自动对焦/手动对焦下长按CAL按键:暂停系统运行，低功耗模式<br>
    + 之后按下REC按键:解除暂停，系统恢复运行<br>

+ 其他:<br>
    + 可能有部分描述与实际代码不符合，存在部分功能的删减和增添，具体功能以实际测试运行为准<br>

### **test platform/测试平台**
- CPU: ryzen7 4800u
- 系统: ubuntu 20.04
- 电池容量: 损耗后剩余46wh
- 测试环境: 关闭wifi和蓝牙，内屏最低亮度，视野内持续追踪单一目标，所有核心限制在最低频率（1.4Ghz）
- 测试结果: cpu核心平均占用约为40%，内存平均占用3.15G(空闲时为3.05G)
           平均帧率维持在17fps上下，较为流畅
           续航时间(99% - 5%)约为2h20min(推算，可能会有浮动)
           以此类推，采用95wh希铁外接电池或99whV口电池可以供电4.5-5h
           采用77wh的F970电池或75wh的希铁外接电池可以供电3.5-4h

### **BOM/物料清单**
+ #### essential components/必备部件
    - **摄影配件**
        - 铁头 原力N电机 / Tilta NucleusN motor
        - 15mm 碳纤维导管 / 15mm carbon fiber tube
        - 15mm 单孔管夹 / 15mm pipe clamp
        - 摄影多孔芝士板 / photography cheese plate
        - 1/4 螺丝 / 1/4 screw

    - **其他配件**
        - 英特尔 D415/D435深度相机 / Intel Realsense D415/D435 depth camera
        - MSCF 滑移支架(固定相机) / MSCF drift support (camera support)
        - cforce拓展坞 / cforce typec dock
        - 相机信号线(usb3.0 typec-a) / camera cable (usb3.0 typec-a)
        - 定制控制手柄信号线 / DIY control handunit cable
        - 全功能typec 主机通信线(长) / USB4 typec (long)
        - 3M 双面胶 / 3M double-side tape

+ #### optional components/选配部件
    - **摄影配件**
        - HDMI 监视器 / HDMI monitor
        - 监视器支架 / monitor support
        - HDMI 公对公线 / HDMI cable
        - 铁头原力N 跟焦手柄 / Tilta NucleusN hand unit
        - 控制手柄连接件 / hand unit adapter    

+ #### cost calculation/成本核算   
  - 组建全新设备成本过高，此处**若无特殊标注，则以2023年8月的闲鱼二手价格**进行计算<br>
  - 必备配件包括：英特尔D415/435深度相机（700r），原力N电机（300r），N100小电脑（全新，900r），连接线和固定件若干（全新，200r）
  - 故**基础成本约为：2100r**
  - 建议选装配件包括：原力N跟焦手柄（500r）、HDMI监视器（500r起）
  - 故**完整成本约为：3100r**
  - 若全部选用全新配件进行组装，则深度相机约为1300r，跟焦手柄约为1100r
  - 故**全新成本约为：4300r**    

