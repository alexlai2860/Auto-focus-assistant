# Auto Focus Assistant

## 辅助对焦系统(原型机)<br>
#### this is an experiment using a depth camera to control a stepping motor, aiming to drive the focus ring and achieve auto focus.<br>

#### 注意:本项目基于GPL3.0协议开源，依据协议要求，后续基于此代码的项目需保持GPL3.0开源<br>

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
+ 另一套方案是本项目当前采取的方案，也是本项目的创新点之一：即采用分布式方案，将算力平台和相机分离，以牺牲集成度的方式大幅降低了成本和重量，并且不需要妥协性能
#### operator/执行器<br>
+ 执行器是直接驱动镜头进行对焦的组件，一般分为内置的对焦马达和外置的跟焦电机
+ 目前所使用的执行器为外置42闭环步进电机，后续可能会更换为28步进电机或无刷电机
+ 同时，基于ESP32进行蓝牙连接部分相机（如BMPCC和ZCAM）的研发也在进行，便于驱动内置马达的自动镜头
+ (2023.4.25)考虑到兼容性和易用性，计划对Tilta NucleusN马达和控制手柄进行逆向研发，分别用于驱动和控制对焦系统
#### controller/控制器<br>
+ 控制器是进行对焦模式切换、实现手自一体对焦的关键部件
+ 本项目从v2.0版本开始加入了控制器，采用Tilta NucleusN控制手柄进行控制
+ 控制逻辑见后文的"使用方法"

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

#### 设置开机自启动<br>
由于本项目需要用到imshow，所以开机自启动的设置会略为复杂，也希望有大佬提出改进建议<br>
若采用传统的systemd去启动程序，由于imshow的存在会报错<br>
故本项目自启动思路如下:将项目做成快捷方式，再用系统自带的开机自启动去启动快捷方式<br>
操作步骤如下<br>
+ 在 /usr/local/下创建一个文件夹，名为af-assistant
+ 进入该文件夹，在文件夹内部创建两个.sh文件，分别命名为af-start.sh和gtk-launch.sh(权限问题不再赘述)
+ 前者用于进入项目文件夹并启动项目代码，后者用于进入快捷方式文件夹并启动快捷方式
+ 前者内容示例如下(可根据代码位置修改)
```
#!/bin/bash

cd /home/alex/AF_assistant/auto-focus-assistant/build
./AF_assistant
```
+ 后者内容示例如下(可根据代码位置修改)
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
+ 最后在桌面应用中，找到名为"启动应用程序"的应用，打开
+ 点击添加，名称可以随意填写，命令填写gtk-launch.sh的地址(/usr/local/af-assistant/gtk-launch.sh)

#### 功率限制<br>
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

#### 控制逻辑<br>
+ NucleusN手柄控制逻辑 & 决策器控制逻辑
按键重映射逻辑如下:<br>
+ 开机和关机
单击开机键开机，长按开机键关机(未改变原始设定)<br>
+ 总模式切换
总模式切换可以在开机后单按REC键(同开机键)实现，同时手柄指示灯颜色改变<br>
总体分为两大模式：自动对焦(绿灯，AF)和手动对焦(红灯，MF)<br>
注意，当模式由自动对焦切换至手动对焦时，默认为锁定状态(用于规避临时障碍物)<br>
只有手动转动滚轮，才能进入全手动模式<br>
+ 自动对焦
    + 滚轮:用于目标切换<br>
      滚轮用于实时选择不同的目标<br>
      选择逻辑如下:<br>
      滚轮对应编码器数值0-9999，通过非线性重映射到实际距离<br>
      通过滚动滚轮可以切换远近目标<br>
      当滚轮放在最左端，默认对焦最近的目标，且不进行追踪<br>
      当滚轮放在最右端，默认对焦至背景(待实现)<br>
    + CAL按键(待实现):用于自动对焦下模式选择<br>
      CAL按键可以切换自动对焦模式，按下按键用于循环切换三种模式<br>
      基于神经网络的人体&人脸混合智能对焦<br>
      中心点对焦<br>
      中心区域对焦<br>
      未来将开放更多的物体识别模式<br>
+ 手动对焦
    + 滚轮:用途和直接驱动原力N电机一致<br>
    + CAL按键:用于常规模式和精准模式切换<br>
      常规模式即将编码器0-9999线性映射至电机读数的0-999<br>
      精准模式即缩小编码器的映射范围<br>

+ 附加功能:<br>
自动对焦/手动对焦下长按CAL按键:暂停系统运行<br>
之后按下REC按键:解除暂停，系统恢复运行<br>

### 测试平台<br>
- CPU: ryzen7 4800u
- 系统: ubuntu 20.04
- 电池容量: 损耗后剩余46wh
- 测试环境: 关闭wifi和蓝牙，内屏最低亮度，视野内持续追踪单一目标，所有核心限制在最低频率（1.4Ghz）
- 测试结果: cpu核心平均占用约为40%，内存平均占用3.15G(空闲时为3.05G)
           平均帧率维持在17fps上下，较为流畅
           续航时间(99% - 5%)约为2h20min(推算，可能会有浮动)
           以此类推，采用95wh希铁外接电池或99whV口电池可以供电4.5-5h
           采用77wh的F970电池或75wh的希铁外接电池可以供电3.5-4h

### 重要更新<br>
+ 2023.5.6
修复此前测试中的bug<br>
修改reprojection函数逻辑，性能大幅提升<br>
新增基于SCRFD的面部识别，速度和Yunet基本一致但侧脸识别率大幅提升<br>

+ 2023.5.3
完成首次室外测试<br>
发现了两个隐藏BUG，正在尝试修复<br>
总体测试效果还行，大部分功能正常<br>
后续方向:修复bug，完善手柄策略(尤其是解决按键识别率的问题)<br>
        给面部识别也加上strict-in策略<br>
其他问题:散热问题较为严重，持续运行超过五分钟机身温度就会非常烫手<br>
        (室外温度接近30度，电脑放置于斜挎包内部)<br>

+ 2023.4.30
四月份最后一次更新<br>
完整demo已经上线，新增电机响应控制、ROI选取和控制手柄策略(部分实现)<br>
只需要一个USB-TTL，拆分为两个半双工模式即可<br>

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
