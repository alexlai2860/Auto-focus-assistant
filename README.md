# Auto Focus Assistant

## 辅助对焦系统(原型机)<br>
##### this is an experiment using a depth camera to control a stepping motor, aiming to drive the focus ring and achieve auto focus.<br>

### 主要原理<br>
本项目属于主动式对焦系统<br>
基于结构光立体视觉技术，通过深度相机获取深度图和彩色图，并将二者进行匹配<br>
通过计算机视觉技术检测并追踪图像中的目标，包括人脸、人体和其他物体，并实时读取目标的距离<br>
通过距离-电机位置映射函数，驱动电机带动对焦环至目标位置，完成对焦动作<br>

### 硬件系统结构<br>
#### range finder/测距仪<br>
+ 用于测量目标与相机的距离，常用的测距仪包括深度相机和各类激光雷达
+ 本项目所使用的测距仪是两款深度相机：奥比中光AstraPro 和 英特尔Realsense D415(兼容435/455/457，未来可能兼容430)
+ 前者的原理的单目红外结构光测距，在室内表现优异，有效距离8m；后者为双目结构光，在室内外均可使用，理论有效距离>10m
+ 由于个人精力有限，大部分工作将基于Realsense相机开发，不能保证对AstraPro提供持续的兼容
#### processor/处理器<br>
+ 用于处理从测距仪得到的数据，常用的视觉处理平台有英伟达的NX系列、树梅派等平台和以NUC为代表的小型电脑
+ 综合考虑成本、体积重量和集成化，本方案所采用的处理器之一为英特尔的Computer Stick(电脑棒)，它采用X86架构，能够方便地进行程序开发和运行
+ 另一套方案是本项目当前采取的方案，也是本项目的创新点只要：即采用分布式方案，将算力平台和相机分离，以牺牲集成度的方式大幅降低了成本和重量，并且不需要妥协性能
#### executer/执行器<br>
+ 执行器是直接驱动镜头进行对焦的组件，一般分为内置的对焦马达和外置的跟焦电机
+ 目前所使用的执行器为外置42闭环步进电机，后续可能会更换为28步进电机或无刷电机
+ 同时，基于ESP32进行蓝牙连接部分相机（如BMPCC和ZCAM）的研发也在进行，便于驱动内置马达的自动镜头
+ (2023.4.25)考虑到兼容性和易用性，计划对Tilta NucleusN马达和控制手柄进行逆向研发，分别用于驱动和控制对焦系统

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
    + FaceDetector:基于卷积神经网络(Yunet)的面部识别，已部署
    + FaceDetectorLight:更为轻量的面部检测网络，待部署
    + ObjectDetector:基于YOLOv5s的目标检测网络，已部署但无法实时检测，暂时搁置
    + ObjectDetectorLight:基于YOLO_fastestV2的目标检测网络，已部署
#### dis/距离获取器<br>
+ 负责距离的解算和滤波
#### motor/电机驱动器<br>
+ 负责与电机间通信
    + 第一代AF_assistant:基于42闭环步进电机
    + 第二代AF_assistant:基于铁头NucleusN电机(在研)
#### reader/读取器<br>
+ 负责从深度相机读取彩色视频流和深度信息流
    + 第一代AF_assistant:基于奥比中光相机
    + 第二代AF_assistant:基于Realsense相机

### 底层<br>
#### serial/通信器<br>
+ 用于和电机/相机/单片机等进行串口通信，目前仅支持电机通信
+ Data类为总协议，分为以下子通信协议：
    + TransferData: 程序内部传递数据规范（其实是大杂烩，并不规范）
    + SendData1/2/3/4/5/6: 程序向电机发送的指令
        + 1~4分别对应步进电机的触发指令、读取指令、修改指令、运动指令
        + 5~6分别对应NucleusN电机的运动指令、校准指令
    + Read: 
        + 对于步进电机:程序从步进电机读取当前脉冲值
        + 对于NucleusN:程序从手柄读取数据
#### param/参数文件<br>
+ 用于注入参数，目前分为以下两类参数：
    + param：普通参数，为定值，运行前可以调整
    + lens_param: 镜头标定参数，储存各个镜头的查找表数据，运行时可能改变（如校准/新建镜头）

### 使用方法<br>
#### 安装依赖库<br>
+ OpenCV 4.5.0或更新版本(推荐4.5.4及以上)
+ Realsense SDK 2.0
#### 运行程序<br>
(在项目文件夹下)<br>
```
mkdir build
cd build
cmake ..
make
./AF_assistant
```
(请安装好深度相机和步进电机，检查齿轮配合情况，最后上电并进行上述步骤)<br>
若电机成功初始化，驱动板应处于不使能状态，此时将对焦环旋转至最近处，随后输入1并按下回车<br>

待完善<br>

### 重要更新<br>
+ 2023.4.25
NucleusN接收和发送数据测试成功，但是:<br>
同时接收手柄数据并发送电机数据需要两个USB-TTL模块<br>
意味着需要同时打开两个串口，需要预先绑定串口<br>
+ 2023.4.20
基本完成决策器和tracker的重写(主要是tracker)<br>
兼容YOLO_facstestv2,具体逻辑待完善<br>
+ 2023.4.09
放弃nuitrack<br>
效果差&SDK烂&还一堆限制<br>
部署 YOLOv5s & YOLO_fastestv2,以后者为主，前者无显卡很难达到实时<br>
准备改进决策系统<br>

+ 2023.4.06
部署nuitrack sdk<br>
这玩意真的过于阴间，此处记录一下安装步骤<br>
    - stage 1: 下载和安装
        - 首先在github上找到nuitrack_sdk，下载Platforms文件夹中对应的安装包（ubuntu amd64），打开并安装
        - 用第三方工具下载其Example和Nuitrack文件夹(完整下载SDK体积会很大)
    - stage 2: 激活和测试
        - 接下来请登陆Nuitrack官网，申请trial license（试用版本），会收到相应包含激活码的邮件
        - 此时打开ubuntu命令行，输入nuitrack，会出现图形界面
        - 随后插入深度相机，激活（会和相机固件绑定）并测试
    - stage 3: 部署必要文件
        - 将Nuitrack文件夹复制/剪切到和Auto Focus Assistant文件夹所在位置相同的位置（作为AF assistant的子文件夹）
        - 将Examples中的multisensor文件夹打开，将它提供的opencv3.4headers解压到Nuitrack文件夹的include目录中
            - (此时include文件夹中应含有opencv子文件夹和opencv2子文件夹)
        
+ 2023.3.15
基本完成顶层模块的重构和函数优化<br>
下一步将继续优化modules，尤其是新增的tools工具库，以及完善奥比中光相机的reader<br>

+ 2023.3.12
取消processor,将原有的processor模块分散至dis、motor、detector、reader、tools等模块<br>
将原FrameProcessor改为FocusController,从原Calibrator分出CalController,加入顶层结构<br>
