# 实用开发小工具

## 目录

- 实用开发小工具
  - [bag_to_image](#bag_to_image)
  - [cmd_vel](#cmd_vel)
  - [edit-txt](#edit-txt)
  - [env-setup](#env-setup)
  - [filter](#filter)
  - [iftop](#iftop)
  - [iotop](#iotop)
  - [latency_test](#latency_test)
  - [license](#license)
  - [loguru](#loguru)
  - [q2euler](#q2euler)
  - [rosbag_topic_size](#rosbag_topic_size)
  - [ros_video_save](#ros_video_save)
  - [select_file](#select_file)
  - [scripts](#scripts)
  - [spdlog](#spdlog)
  - [stereo_calib_markerless](#stereo_calib_markerless)
  - [usbtop](#usbtop)
  - [udp2can](#udp2can)
  - [utils](#utils)
  - [voc_to_yolo](#voc_to_yolo)
  - [yalk](#yalk)

## bag_to_image

**bag_to_image.py** ,**bag_to_depth.py** **bag_to_image_and_imu.py**分别是将bag中的topic转成RGB图, 深度图, 双目图+IMU的脚本

## cmd_vel

### 用途

使用键盘来发送cmd_vel命令

### 使用

将脚本文件(二选一)添加到ROS功能包的目录下，修改所需的目标topic name，编译完成后，使用以下命令：

```
rosrun your_package_name sl_bigcar_keyboard_teleop.py
```

## edit-txt

### 用途

用于修改yolov5的训练label格式

- *edit_txt.py* : 修改文件夹中的所有txt文件的其中一列的数值
- *delete_special_line.py* : 删除文件夹中的所有txt文件中某一列含有数值n的行
- *change_num_txt.py*：改变第一列的类别值

### 使用

```
# 修改文件夹名
python edit_txt.py
# 刪除多余的类别
python delete_special_line.py
#将类別按照自己的方法编号
python change_num_txt.py
```

## env-setup

### 用途

安装各种软件包的自动化脚本

### 使用

```
cd your_path/env-setup/
./packages/install_package_name.sh
```

## filter

### 用途

使用滤波对数据进行过滤

### 使用

```c++
#include "single_kalman.h"
int main(int argc, char **argv) {
    SingleKalmanFilter singleKalmanFilter(1, 1, 1, 0, 1);
    singleKalmanFilter.setProcessNoise(0.01);
    singleKalmanFilter.setMeasurementNoise(15.4);

    double data[10] = {1,2,3,4,5,6,7,8,9,10}
    for(int i = 0; i < 10; i ++){
        double single_filter = singleKalmanFilter.filter(data[i], 0);
        std::cout << single_filter << std::endl;
    }
    return 0;
}
```

## iftop

### 用途: 

监控网卡的实时流量

### 安装:

所需依赖:

```
$ sudo apt-get install flex byacc  libpcap0.8 libncurses5
```

```
$ wget http://www.ex-parrot.com/pdw/iftop/download/iftop-0.17.tar.gz
$ tar zxvf iftop-0.17.tar.gz
$ cd iftop-0.17
$ ./configure
$ make
$ sudo make install
```

### 使用:

- 使用**ifconfig**查看网卡号

- ``` 
  $ sudo iftop -i enp109s0
  ```

#### 常用参数:

- -i设定监测的网卡，如：# iftop -i eth1

- -B 以bytes为单位显示流量(默认是bits)，如：# iftop -B

- -n使host信息默认直接都显示IP，如：# iftop -n

- -N使端口信息默认直接都显示端口号，如: # iftop -N

- -F显示特定网段的进出流量，如# iftop -F 10.10.1.0/24或# iftop -F 10.10.1.0/255.255.255.0

- -h（display this message），帮助，显示参数信息

- -p使用这个参数后，中间的列表显示的本地主机信息，出现了本机以外的IP信息;

- -b使流量图形条默认就显示;

- -f这个暂时还不太会用，过滤计算包用的;

- -P使host信息及端口信息默认就都显示;

- -m设置界面最上边的刻度的最大值，刻度分五个大段显示，例：# iftop -m 100M

#### iftop界面相关说明

界面上面显示的是类似刻度尺的刻度范围，为显示流量图形的长条作标尺用的。

中间的<= =>这两个左右箭头，表示的是流量的方向。

TX：发送流量
RX：接收流量
TOTAL：总流量
Cumm：运行iftop到目前时间的总流量
peak：流量峰值
rates：分别表示过去 2s 10s 40s 的平均流量

## iotop

### 用途

iotop 是一个用来监视磁盘I/O使用状况的top类工具。iotop具有与top相似的UI，其中包括PID、用户、I/O、进程等相关信息。

### 安装

```
sudo apt-get install iotop
```

### 使用

```
sudo iotop
```

### 参数说明

- -o：只显示有io操作的进程
- -b：批量显示，无交互，主要用作记录到文件
- -n NUM：显示NUM次，主要用于非交互式模式
- -d SEC：间隔SEC秒显示一次
- -p PID：监控的进程pid
- -u USER：监控的进程用户

### 常用快捷键

- 左右箭头：改变排序方式，默认是IO排序
- r：改变排序顺序
- o：只显示有IO输出的进程
- p：进程、线程的显示方式的切换
- a：显示累积使用量
- q：退出

## latency_test

### 用途

测试发送端和接受端的时延

### 使用

```
./send
./receive
```

## license

### 用途

生成验证文件

### 使用

```
./generate_license (默认)
./generate_license --public_key=/your_path/public.pem --private_key=/your_path/private.pem  (可选)
```

## loguru

### 用途

日志记录工具

### 安装

```
# python
pip install loguru

# C++
将loguru.cpp和loguru.hpp复制到项目目录下
```

### 使用

```
# python
from loguru import logger

# C++
#include "loguru.hpp"
```

## q2euler

### 用途

欧拉角和四元数互相转换

## rosbag_topic_size

### 用途: 

查看录制的bag中每种topic的消息size, 检测异常topic

### 使用:

```
$ python rosbag_topic_size.py bag_name.bag
```

## ros_video_save

### 用途

- 将topic数据存储为视频文件
- 将视频文件转为bag文件

### 使用

```
source devel/setup.bash
#或者
source devel/setup.zsh
# topic转成视频
rosrun ros_video_save ros_video_save

# 视频转成bag
cd your_path/ros_video_save/script
chmod a+x video2bag.py
python video2bag ../data/write.avi bag_name.bag
```

## select_file

### 用途

- 随机/自定义选取文件夹中的图片移动/复制到另一个文件夹中
- 给文件夹内的文件重命名

### 使用

- 随机选取图片

```
python select_file.py
```

- 文件重命名(随机排序)

```
python rename.py
```

## scripts

### 用途

包含各种脚本

- **perf.sh**：分析程序的CPU/内存占用率
- **kill.sh**：杀死某一类进程

## spdlog

### 用途

- 日志记录工具

### 安装

```bash
sudo apt install libspdlog-dev

# 编译安装
git clone https://github.com/gabime/spdlog.git
cd spdlog && mkdir build && cd build
cmake .. && make -j
```

### 使用

```C++
#include "spd_logger.h"
```

## stereo_calib_markerless

### 用途

在无标定板的情况下进行双目相机的外参标定

### 依赖

- OpenCV3
- Eigen
- Ceres
- boost
- Threads

### 文件说明

- **stereo_calib_cheeseboard.cpp**：单张标定板图片外参标定
- **stereo_calib_no_cheeseboard.cpp**：单张无标定板图片外参标定
- **stereo_calib_cheeseboard_multi.cpp**：多张标定板图片外参标定
- **stereo_calib_no_cheeseboard_multi.cpp**：多张无标定板图片外参标定
- **test_cheeseboard.cpp**：使用优化后的外参在标定板图片上测试
- **test_no_cheeseboard.cpp**：使用优化后的外参在无标定板图片上测试

### 使用

1. git clone文件

2. 将其放入工作空间内**catkin_ws**

3. 更改内参（ *left_camera* ，*right_camera*）、外参 （*LoadExtrinsics*）、图片 （*BASE*） 文件路径，如果使用多图片标定，还要更改图片数量 （*g_picture_num*）、扰动值 （*g_noise*）

4. 编译功能包

```
catkin_make -DCATKIN_WHITELIST_PACKAGES=stereo_calib_markerless
```

5. 更新环境

 ```
  source ~/devel/setup.bash
  或者
  source ~/devel/setup.zsh
 ```

6. 运行

```
rosrun stereo_calib_markerless stereo_calib_no_cheeseboard_multi  # 多图片标定
```

```
roscore
rviz
rosrun markerless_stereo_calib show_pcd_cheese
```

## usbtop

### 用途: 

监控USB 类传感器的使用带宽

### 依赖

- libpcap
- libboost>=1.48.0

```
sudo apt install libboost-dev libpcap-dev
```

### 安装

命令行安装: 

```
$ sudo apt install usbtop
```

从源文件安装:

```
$ cd /path/to/usbtop
$ mkdir _build && cd _build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
$ sudo make install
$ sudo modprobe usbmon   --------最后一定要加这句,加载模块
```

使用:

```
$ sudo usbtop
```

## udp2can

### 用途

解析canet的can数据

### 使用

```
# 修改端口号
./udp2can
```

## utils

### 用途

包含各种小工具

- json：C++数据格式转成json格式
- net：检测IP工具
- obfuscate：字符加密
- perf：计算耗时
- time：获取时间戳（秒、毫秒）

## voc_to_yolo

### 用途

将voc格式的标注文件修改成yolo格式

### 使用

```
# 先替换文件夹路径
cd ~/your_path/voc_to_yolo
python voc2yolo.py
```

## yalk

### 用途

加密软件

### 使用

```shell
./build.sh
#修改将公钥和许可证路径后
./build.sh
```

1. 在开发者主机中，使用 YALK 项目中构建的命令行工具 `clis_yalk` 生成密钥对，并将公钥字符串复制到文件 `samples/simple_demo/include/yalk_helper.h` 。

   ```bash
   ./clis_yalk keygen /tmp
   Public key saved to /tmp/public_key.pem
   Private key saved to /tmp/private_key.pem
   Base64 encoded public key:
   ...
   Base64 encoded private key:
   ...
   ```

2. 在客户端，使用 YALK 项目中构建的命令行工具 `clis_yalk` 生成机器指纹文件

   ```bash
   ./clis_yalk collect /tmp/public_key.pem -o /tmp/machine.dat
   HWID saved to /tmp/machine.dat
   ```

   或者不指定输出文件名，直接输出经过 Base64 编码的机器指纹字符串，以便传输：

   ```bash
   ./clis_yalk collect /tmp/public_key.pem -o /tmp/machine.dat
   HWID (encoded):
   ...
   ```

   或者在现有项目中直接调用`yalk_collect`函数生成机器指纹文件。

3. 在开发者主机中，使用 YALK 项目中构建的命令行工具 `clis_yalk` 生成许可证文件

   ```bash
   $ ./clis_yalk issue /tmp/private_key.pem /tmp/machine.dat -o /tmp/yalk.lic
   
   Config:
   {"common":{"issue_number":0,"product_name":"YALK","product_version":"1.0.0","type":0}}
   Issued license saved to /tmp/yalk.lic
   ```

   或者在现有项目中直接调用`yalk_issue`函数生成许可证文件。

4. 在客户端，项目中直接调用`yalk_verify`函数，对给定的许可证文件（在第 3 步中生成）、本机的机器指纹（运行时采集）以及请求的许可证限制（预设于`yalk_helper.h`中）三者进行验证。
