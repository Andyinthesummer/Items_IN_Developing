## Operations In Deployment

### 零 准备工作

#### 0.1 工具设备

1. 笔记本一台. 用于远程数据录制和设备调试

    系统要求: `ubuntu 16.04 + ROS kinetic` OR `ubuntu 18.04 + ROS melodic`

    ROS安装: 根据`Ubuntu_DISTRO`选择相应的[ROS_DISTRO](http://wiki.ros.org/Distributions)进行安装。

2. 4G流量卡一张. 给巡检车提供移动网络，方便巡检车同管理后台和手机APP通信

    安装方法: 参考 **4G卡更换说明文档**。

### 一 开机检查

巡检车启动方式以及遥控器控制的具体使用方法，请参考 **巡检车使用流程**。

#### 1.1 无线网络信息

* Wifi 名称: `car_3`

* Wifi 密码: `gzwx2020`

* 后台管理地址: http://huaweilogin.com (失败则采用 `IP` 地址 `192.168.1.1` 访问)

* 后台管理密码: `gzwx2021`

#### 1.2 设备IP信息

* 导航工控机: `192.168.1.101` 

* 服务工控机: `192.168.1.108`

* VLP-Lidar: `192.168.1.201`

* SLidar: `192.168.10.200`

* GPS-Module: `192.168.10.100`

#### 1.3 设备检查

* **Note :** 检查过程中如若发现问题，可重新启动整个巡检车并再次按流程进行检查，以定位问题。

* **Tips :** Linux终端基本操作, 在后续操作中会频繁使用:

    * 打开新终端: `ctrl + alt + t`
    * 关闭终端: `ctrl + d` 
    * 终止终端运行的程序: `ctrl + c`
    * 终端命令补充: `Table`

* 启动巡检车，稍作等待，让笔记本连接上巡检车`wifi`，打开`Terminal`，检查设备是否在线，具体操作如下:

    ```bash
    $ ping 192.168.1.101 ## 检查 导航工控机
    $ ping 192.168.1.108 ## 检查 服务工控机
    $ ping 192.168.1.201 ## 检查 VLP Lidar
    $ ping 192.168.10.100 ## 检查 SLidar
    $ ping 192.168.10.200 ## 检查 GPS Module
    ```

* `ssh`远程登录到 *导航工控机* ,检查设备是否正常连接，具体操作如下

    ```bash
    ## 登录 导航工控机
    ## user: mind
    ## password: mind
    $ ssh mind@192.168.1.101 
    ## 打开自启文件，
    $ vim .config/autostart/auto_startup.sh
    ## 将debug_state修改为true，保存后退出
    ## 重新启动 导航工控机
    $ sudo reboot 
    ## 等待一分钟，再次登录 导航工控机
    $ ssh mind@192.168.1.101 
    ## 检查IMU
    $ ls /dev/LP_IMU
    ## 检查GPS
    $ ls /dev/GPS
    ## 检查底盘,是否有正常数据输出
    $ candump can0
    ## ctrl + c 关闭输出 
    ```

### 二 数据录制

#### 2.1 数据录制准备

* **Note :** 数据录制前，最好同技术人员确认，场景是否适合，比如可以将预录制场景的照片、googlemap等发给相关技术人员

* 设备检查
    * 通过 ***开机检查*** 中所有的设备测试

* 场景选择 
    * 适合室外
    * 转弯半径不小于`3m`
    * 道路宽度不小于`3m`
    * 整个场景中不存在较大的高度差 
    * 录制面积不能过大
    * 如若使用GPS,则场地周边不能有过多树木和大楼遮挡

* 移动控制
    * 通过遥控器控制移动，尽量匀速移动
    * 速度控制在`0.5m/s-1.0m/s`
    * 注意激光雷达的扫描范围，为正前方180度，注意让雷达覆盖尽量大的范围



#### 2.2 数据录制

* **Note :**

    * 录制数据的笔记本必须一直跟在巡检车后面，防止`wifi`断连，造成数据录制失败
    * 录制时，跟随人员不要长时间出现在巡检车激光雷达的范围内

* 将巡检车移动到要录制的场景中，并选择**合适的起点**(决定了**巡检车的初始位置和方向**)

    * 该起点很重要，尤其在无GPS的情况下，后续**启动巡检车**都必须要在此处进行

* 笔记本连接到巡检车`wifi`, 打开`Terminal`**1**并`ssh`远程登录到 *导航工控机* , 并启动数据节点，具体操作如下:

    ```bash
    $ ssh mind@192.168.1.101
    $ roslaunch car_bringup startup_record_databag.launch
    ## 注意检查控制台输出有无报错(报错一般为红色信息)
    ```

* 笔记本打开新的终端`Terminal`**2**，检查数据是否正常输出，具体操作如下:

    ```bash
    $ ssh mind@192.168.1.101
    ## 依次检查数据输出是否正常
    $ rostopic echo /velodyne_points
    $ rostopic echo /scan
    $ rostopic echo /nmea_sentence
    $ rostopic echo /odom
    $ rostopic echo /imu/data
    ## ctrl + c 关闭输出
    ```
* 笔记本打开新的终端`Terminal`**3**，进行数据录制，具体操作如下:

    ```bash
    $ ssh mind@192.168.1.101
    ## 运行脚本，进行数据录制
    $ ./ros_ws/src/car_bringup/scripts/recordbag.sh
    ## 注意查看，数据录制是否出错
    ```

* 笔记本打开新的终端`Terminal`**4**，查看正在录制的数据，具体操作如下:

    ```bash
    $ ssh mind@192.168.1.101
    ## 查看正在录制的数据,并能够得知 当前已录数据包大小
    $ cd databag
    $ ll -h sensors_data*
    ## 根据时间，可以得知当前正在录制的数据包信息
    ```
* 匀速控制车辆，直到数据录制结束，`ctrl + c`关闭录制数据的`Terminal`**3**，并关闭其他所有终端。

#### 2.3 数据回传

* 笔记本打开新的终端`Terminal`，将数据回传到笔记本，具体回传操作如下:
    ```bash
    ## 会传到笔记本home目录下
    $ scp mind@192.168.1.101:~/databag/sensors_data* .
    ```

* 请将回传的数据上传到`google drive`或者百度网盘等，并分享给相关技术人员进行地图制作。

### 三 地图应用

#### 3.1 地图上传

* 连接巡检车，打开终端，将地图上传至巡检车，具体操作如下:

    ```bash
    ## vector map, 注意文件名，根据实际更改
    $ scp -r sig_first mind@192.168.1.101:~/ros_ws/src/car_bringup/map/vector_maps
    ## pcd map, 注意文件名，根据实际更改
    $ scp sig_first.pcd mind@192.168.1.101:~/ros_ws/src/car_bringup/map/point_cloud_maps/
    ```
#### 3.1 地图应用

* 打开新终端，并`ssh`远程登录到 *导航工控机* , 将地图变更应用到配置文件中，修改具体操作如下:
    ```bash
    $ ssh mind@192.168.1.101
    ## 打开要修改的文件
    $ rosed car_bringup startup_map_loader.launch
    ```
* 修改如下字段:

    * 将文件中下行的`yard_bin1030.pcd`修改为`sig_first.pcd`(请根据实际的文件名修改)
    ```xml
    <arg name="pcd_paths" default="$(find car_bringup)/map/point_cloud_maps/yard_bin1030.pcd" /> <!-- bin_autoware1030_change_tf.pcd  -->
    ```

    * 将将文件中下行的`yard_0203`修改为`sig_first`(请根据实际的文件名修改)
    ```xml
    <arg name="vector_path" default="$(find car_bringup)/map/vector_maps/yard_0203/" />
    ```


### 四 启动运行

* 笔记本连接`wifi`,`ssh`远程登录到 *导航工控机* ,设置开机自启(仅在初次使用时操作)，具体操作如下:

    ```bash
    $ ssh mind@192.168.1.101 
    ## 打开自启文件
    $ vim .config/autostart/auto_startup.sh
    ## 将debug_state修改为false,保存后退出
    ```

* 将巡检车用遥控器移动到数据录制中的**初始位置**，位置和方向不能有太大偏差，重新启动巡检车后，即可正常操作，具体细节参考 **巡检车使用说明**

