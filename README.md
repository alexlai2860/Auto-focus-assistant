# sleep-monitor
### 深度相机-睡眠检测<br>
#### 主要原理<br>
首先使用官方例程，用两个线程分别读取彩色视频流和深度视频流<br>
随后找到对彩色视频流进行面部检测，此处使用了opencv4.5.4新加入的yunet,较为轻量，适合后续在移动设备上部署<br>
通过检测计算出面部中心和关键点，判断面部的姿态并计算中心距离<br>
根据面部姿态和距离，评估目标的翻身频率 & 睡眠姿态，和其他传感器一起评估睡眠时间 & 睡眠质量等指标<br>
<br>

### 衍生项目-辅助对焦系统(原型机)<br>
##### this is an experiment using a depth camera to control a stepping motor, aiming to drive the focus ring and achieve auto focus.<br>
#### 主要原理<br>
通过解算目标面部/眼部到相机的距离<br>
根据距离-电机位置拟合的函数（Mat类型，以图片文件形式储存）<br>
向电机发送目标值与当前值的差值<br>
从而驱动电机抵达目标位置<br>



### 使用方法<br>
#### 睡眠检测系统<br>
(在项目文件夹下)<br>
```
mkdir build
cd build
cmake ..
make
./main
```
#### 辅助对焦系统(原型机)<br>
(请安装好深度相机和步进电机，检查齿轮配合情况，最后上电并进行上述步骤)<br>
若电机成功初始化，驱动板应处于不使能状态，请将对焦环旋转至最近处，随后输入1并按下回车<br>



### 更新记录
<br>

