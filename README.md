# Auto Focus Assistant

## 辅助对焦系统(原型机)<br>
##### this is an experiment using a depth camera to control a stepping motor, aiming to drive the focus ring and achieve auto focus.<br>

### 主要原理<br>
首先使用官方例程，用两个线程分别读取彩色视频流和深度视频流<br>
随后找到对彩色视频流进行面部检测，此处使用了opencv4.5.4新加入的yunet,较为轻量，适合后续在移动设备上部署<br>
通过检测计算出面部中心和关键点，判断面部的姿态并计算中心距离<br>
根据距离-电机位置拟合的函数（Mat类型，以图片文件形式储存）<br>
向电机发送目标值与当前值的差值<br>
从而驱动电机抵达目标位置<br>

### 硬件系统结构<br>
#### range finder/测距仪<br>
+ 用于测量目标与相机的距离，常用的测距仪包括深度相机和各类激光雷达
+ 本项目所使用的测距仪是两款深度相机：奥比中光AstraPro 和 英特尔Realsense D415(兼任435/455/457，未来可能兼容430)
+ 前者的原理的单目红外结构光测距，在室内表现优异，有效距离8m；后者为双目结构光，在室内外均可使用，理论有效距离>10m
+ 由于个人精力有限，大部分工作将基于Realsense相机开发，不能保证对AstraPro提供持续的兼容
#### processor/处理器<br>
+ 用于处理从测距仪得到的数据，常用的视觉处理平台有英伟达的NX系列、树梅派等平台和以NUC为代表的小型电脑
+ 综合考虑成本、体积重量和集成化，本方案所采用的处理器之一为英特尔的Computer Stick(电脑棒)，它采用X86架构，能够方便地进行程序开发和运行
+ 另一套方案是本项目的专利核心，即采用分布式方案，将算力平台和相机分离，以牺牲集成度的方案大幅降低了成本和重量，并且不需要妥协性能
#### executer/执行器<br>
+ 执行器是直接驱动镜头进行对焦的组件，一般分为内置的对焦马达和外置的跟焦电机
+ 目前所使用的执行器为外置42闭环步进电机，后续可能会更换为28步进电机或无刷电机
+ 同时，基于ESP32进行蓝牙连接部分相机（如BMPCC和ZCAM）的研发也在进行，便于驱动内置马达的自动镜头

### 文件结构（由高到底排序）<br>

### 上层<br>
#### main<br>
+ main函数：程序入口
#### controller/控制器<br>
+ 控制程序运行,负责切换校准模式和对焦模式
#### SubController/子控制器<br>
+ 以下两个控制器的基类
#### CalController/校准模式控制器<br>
+ 在校准模式下，调用控制各个modules
#### FocusController/对焦模式控制器<br>
+ 在对焦模式下，调用控制各个modules

### 核心层<br>
#### modules/核心模块:分为以下子模块<br>
#### calibrator/校准器<br>
+ 负责对镜头进行校准，建立查找表并写入lens_param.yml
#### detector/识别器<br>
+ 负责识别目标
    + FaceDetector:基于卷积神经网络的面部识别
    + 后续计划添加BodyDetector等目标检测算法
#### dis/距离获取器<br>
+ 负责距离的解算
#### motor/电机驱动器<br>
+ 负责与电机间通信
#### reader/读取器<br>
+ 负责从深度相机读取彩色视频流和深度信息流
+ 奥比中光相机和Realsense相机分别拥有不同的Reader

### 底层<br>
#### serial/通信器<br>
+ 用于和电机/相机/单片机等进行串口通信，目前仅支持电机通信
+ Data类为总协议，分为以下子通信协议：
    + TransferData: 程序内部传递数据规范
    + SendData1/2/3/4: 程序向电机发送的指令，分别对应触发指令、读取指令、修改指令、运动指令
    + ReadPulse: 程序从电机读取当前脉冲值
#### param/参数文件<br>
+ 用于注入参数，目前分为以下两类参数：
    + param：普通参数，为定值，运行前可以调整
    + lens_param: 镜头标定参数，储存各个镜头的查找表数据，运行时可能改变（如校准/新建镜头）

### 使用方法<br>
#### 安装依赖库<br>
+ OpenCV 4.5.0或更新版本
+ Realsense SDK 2.0
#### 运行程序<br>
(在项目文件夹下)<br>
```
mkdir build
cd build
cmake ..
make
./main
```
(请安装好深度相机和步进电机，检查齿轮配合情况，最后上电并进行上述步骤)<br>
若电机成功初始化，驱动板应处于不使能状态，此时将对焦环旋转至最近处，随后输入1并按下回车<br>

待完善<br>

### 重要更新<br>
+ 2023.3.15
基本完成顶层模块的重构和函数优化<br>
下一步将继续优化modules，尤其是新增的tools工具库，以及完善奥比中光相机的reader<br>

+ 2023.3.12
取消processor,将原有的processor模块分散至dis、motor、detector、reader、tools等模块<br>
将原FrameProcessor改为FocusController,从原Calibrator分出CalController,加入顶层结构<br>
