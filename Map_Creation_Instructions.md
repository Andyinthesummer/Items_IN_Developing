## 地图构建说明

### 0 准备工作

* 完成数据录制
  
* 打开`Terminal`并`ssh`远程登录到 *导航工控机*，将巡检车修改为**调试模式**:
  
    ```bash
    $ ssh mind@192.168.1.101 
    ## 打开配置文件
    $ vim .config/autostart/auto_startup.sh
    ## 将debug_state修改为true，
    ## 且record_data为false
    ## align_mapgnss为false，保存后退出
    $ sudo reboot
    ```

* 如果要进行`GPS`坐标对齐，数据中的`GPS`信息必须是有效的。

* 建立笔记本和巡检车之间的通信连接

    系统要求: `ubuntu 18.04 + ROS melodic`

    ROS安装: 根据`Ubuntu_DISTRO`选择相应的[ROS_DISTRO](http://wiki.ros.org/Distributions)进行安装。

    其他依赖安装：
    ```shell
    sudo apt-get install ros-melodic-autoware-*
    sudo apt-get install ros-melodic-jsk-rviz-plugins
    ```

    在笔记本`~/.bashrc`文件末尾添加如下字段,建立通信通道:

    ```shell
    export ROS_MASTER_URI=http://192.168.1.101:11311
    export ROS_IP=192.168.1.103 
    ```
    ***Note***: `ROS_IP`为笔记本在巡检车`wifi`下的`IP`，请根据实际情况修改


### 1 点云地图构建

**Replay the databag, and create pointcloud map**

- `Terminal`**1**，并`ssh`远程登录到 *导航工控机*，运行建图程序

    ```bash
    $ ssh mind@192.168.1.101 
    $ roslaunch car_bringup startup_ndt_slam.launch
    ```

- `Terminal`**2**，并`ssh`远程登录到 *导航工控机*，回放数据：
    
    `databag_name.bag`为录制场景的数据包，可以根据录制时间进行区分。

    ```bash
    $ ssh mind@192.168.1.101 
    $ cd databag
    $ rosbag play -r 0.5 --clock databag_name.bag --topics /velodyne_points
    ## ndt mapping will consume a lot of time and computation resources, 
    ## we must keep the time provider alive or reduce the rate of databag player
    ## or -k， --keep-alive
    ## or, -r FACTOR, --rate=FACTOR

    ```

    *Note :* 加入 `-r 0.5` 参数控制数据回放速度，减轻工控机资源占用压力

-  `Terminal`**3**，通过`RVIZ`进行可视化,实时观测地图构建过程
  
    You can tarck the process of ndt mapping in the terminal and startup rviz tool for visualization: 

    `car_test.rviz`为提前配置好的可视化文件
    
    ```bash
    $ rviz -d car_test.rviz
    ```

-  `Terminal`**4**，save the pointcloud map

    While ndt mapping is done, you can save the pointcloud map as following:

    ```bash
    $ rostopic pub -1 /save_ndtmap std_msgs/Bool "data: true"
    ```

    点云地图保存在巡检车`/home/mind/mapfile`文件夹下，根据创建时间进行命名，为`ascii`格式文件，。

    至此完成点云地图构建，关闭所有`Terminal`。


### 2 矢量地图构建

**convert pcd map to binary format**

矢量地图构建需要`binary`格式的文件，可以通过如下方法进行转换: 

`ascii.pcd`为上面创建的点云地图，`bin.pcd`为构建矢量地图所需要的二进制地图

```bash
$ ssh mind@192.168.1.101 
$ ./tool/ascii2bin ascii.pcd bin.pcd ## 根据具体文件名进行修改
```

利用基于 `Unity` 的开发的工具[MapToolbox](https://github.com/Andyinthesummer/MapToolbox)(原项目的`fork`)进行创建，具体请参考项目的视频。

**Requirements**

* binary format pcd file

* Windows System
* [Unity 2019.4](https://store.unity.com/download?ref=personal)
* [Git](https://www.git-scm.com/download/)

**Compile**

* Create your new project in Unity 2019.4
* Add two lines below to Packages/manifest.json dependencies
  
``` json
"com.autocore.map-toolbox": "https://github.com/Andyinthesummer/MapToolbox.git",
"com.nition.unity-octree": "https://github.com/autocore-ai/UnityOctree.git#upm",
```

*Note :*     

* 通过`unity hub`安装`unity`个人版
* 矢量地图编辑完成导出，并在`vector_map_loader`和`op`中加载成功，保存该工程后关闭再次打开，重新导出后在`op`中加载可能出错，目前是先保留一条`lane`导出一次，再`ctrl+z`撤回，再次导出问题不再出现。
* 矢量地图的密度插件不能修改
* 矢量地图构建可以使用在`github`上`fork`的版本进行编辑创建。


**OUTPUT**
输出一个包含数个`csv`文件的文件夹，准备好之后将上传至巡检车中。

### 3 WEB可视化地图制作

制作用于`web`显示的`2D`地图。

* 打开`Terminal`并`ssh`远程登录到 *导航工控机*，将巡检车修改为**调试模式**:
  
    ```bash
    $ ssh mind@192.168.1.101 
    ## 打开配置文件
    $ vim .config/autostart/auto_startup.sh
    ## 将debug_state修改为true，
    ## 且record_data为false
    ## align_mapgnss为false，保存后退出
    $ sudo reboot
    ```

* 打开`Terminal`**1**并`ssh`远程登录到 *导航工控机*，将要操作的点云文件路径添加到目标程序中:

    ```bash
    $ ssh mind@192.168.1.101
    $ rosed car_bringup pcd_to_planemap.launch 
    ```

    `path`值修改为实际点云文件路径:

    ```xml
    <param name="path" value="/home/mind/.config/autostart/param_config/sig_813_2.pcd" type="str" />
    ```
    保存后退出。

* 在`Terminal`**1**工控机上运行转换程序
  
    ```bash
    ## 生成地图
    $ roslaunch car_bringup pcd_to_planemap.launch 
    ```

* 打开`Terminal`**2**，笔记本上运行可视化程序：
  
  ```bash
  rviz -d OctomapShow.rviz
  ```

    通过rviz可视化界面可以查看构建`2D`地图的效果

* 运行指令，保存地图
  
    ```bash
    $ rosrun map_server map_saver map:=/projected_map -f sig_first
    ```

`-f` 参数指定`2D`地图名，生成的地图包含`pgm`和`yaml`两个文件。

**Convert pgm map to png for web visulization**

`ssh`远程登录到 *导航工控机* , 利用脚本转化为`png`格式：

```bash
$ ssh mind@192.168.1.101 
$ python ./tool/pgmConvert.py -i sig_first.pgm -o sig_first.png  -f png
```

**将`yaml`参数和`png`配置到`web`显示中**

建图后可使用`GIMP` `PS`等工具对`png`地图进行美化、修整、固定点标识、路线绘制等，不影响定位等功能，只用于WEB显示。

### 4 GPS 坐标对齐

**回放数据获取`GPS`参考原点**

*terminal 1*

    ```bash
    $ ssh mind@192.168.1.101
    roscore
    ```

*terminal 2*

    ```bash
    $ ssh mind@192.168.1.101
    $ rostopic echo /nmea_sentence |grep sentence
    ```

*terminal 3*

    ```bash
    $ ssh mind@192.168.1.101
    $ cd databag
    $ rosbag play -r 0.1 databag_name.bag --topics /nmea_sentence
    ```

`terminal 2`有数据后，即可`ctrl + c`断开，取最开始的两帧数据，如下所示：

    ```yaml
    sentence: "$GPGGA,021745.90,0119.7210043,N,10344.5923406,E,4,10,0.5,17.586,M,9.570,M,1.9,0258*49"
    sentence: "$PASHR,HPR,021745.90,302.15390,1.90221,,0.0065,0.0000,0,2,0.900,1.2*11"
    ```

逗号分割数据定义：0-GPGGA 1-021745.90 2-0119.7210043 ......

提取`GPS`参考原点：

    ```yaml
    origin_latd： 0119.7210043 ## GPGGA中逗号分割的第2个数据
    origin_lond： 10344.5923406 ## GPGGA中逗号分割的第4个数据
    origin_height： 17.586 ## GPGGA中逗号分割的第9个数据
    origin_yaw： 302.15390 ## PASHR中逗号分割的第3个数据
    ```

关闭所有终端，打开新终端，并`ssh`远程登录到 *导航工控机* , 将`GPS`参考原点的值更新到配置文件对应参数中：


    ```bash
    $ ssh mind@192.168.1.101
    ## 打开要修改的文件
    $ vim .config/autostart/nav_parameters.yaml
    ```

    ```yaml
    ## gnss reference origin
    nmea2tfpose:
        origin_latd: 0119.7210043
        origin_lond: 10344.5923406
        origin_height: 17.586
        origin_yaw: 302.15390
    ```

**上传更新新创建的地图**

```bash
## vector map: 一个文件夹, 包含数个csv文件
## pcd map: 单个文件，以.pcd结尾
## 将vector map文件夹sig_first传到目标位置，文件夹名根据实际更改
$ scp -r sig_first mind@192.168.1.101:/home/mind/.config/autostart/param_config/vector_maps/
## 将pcd map文件传到目标位置, 文件名根据实际更改
$ scp sig_first.pcd mind@192.168.1.101:/home/mind/.config/autostart/param_config/point_cloud_maps/
```

* 打开新终端，并`ssh`远程登录到 *导航工控机* , 将地图变更应用到配置文件中，修改具体操作如下:

    ```bash
    $ ssh mind@192.168.1.101
    ## 打开要修改的文件
    $ vim .config/autostart/nav_parameters.yaml
    ```

* 配置地图:

    * 将文件中下行的`sig_813_2_bin.pcd`修改为`sig_first.pcd`(请根据实际的文件名修改)
    ```yaml
    pcd_paths: [/home/andy/.config/autostart/param_config/point_cloud_maps/sig_813_2_bin.pcd]
    ```

    * 将将文件中下行的`sig_0813_2`修改为`sig_first`(请根据实际的文件名修改)
    ```yaml
    map_dir: /home/andy/.config/autostart/param_config/vector_maps/sig_0813_2
    ```

**进行坐标对齐操作**

* 将巡检车切换到**坐标对齐模式**
    ```bash
    $ ssh mind@192.168.1.101 
    ## 打开配置文件，
    $ vim .config/autostart/auto_startup.sh
    ## 将debug_state修改为true，
    ## 且record_data修改为false,
    ## align_mapgnss保持为true，保存后退出

    ## 重新启动 导航工控机
    $ sudo reboot
    ```

* 等待1分钟，打开`Terminal`**1**,运行可视化程序：

    ```bash
    $ rviz -d car_test.rviz
    ```

    已经预先将`/ndt_pose` `gnss_pose_fixed`加入到显示中，请注意这两个`pose`话题，对齐的目标就是让这两个`pose`基本保持一致。

* 打开`Terminal`**2**,运行对齐界面：
  
  ```bash
  rosrun rqt_reconfigure rqt_reconfigure 
  ```
  
  将界面上的 `tf_x` 和 `tf_y` 以及 `tf_rot_z`三个参数设置为 `0`

* 回放对应场景的数据
      ```bash
    $ ssh mind@192.168.1.101 
    $ cd databag
    $ rosbag play --clock databag_name.bag
    ```

**Adjustment**

通过观察`Terminal`**1**中`ndt_pose`和`gnss_pose_fixed`的对齐情况，或者在`Terminal`中直接获取两个`pose`的偏差：
```bash
rostopic echo /diff_ndt_gnss |grep data:
```
`data: [-2.48571515083313, -0.3461196720600128, -0.007701839320361614]`分别对应`tf_x` 和 `tf_y` 以及 `tf_rot_z`的偏差。

根据偏差，先调整`Terminal`**2**中的方向`tf_rot_z`参数，让方向基本一致，再调整位置`tf_x`和`tf_y`，通过逐步调整参数，实现两个`pose`基本对齐。

*Tips :* 数据播放完成后可以再次播放，从而可以循环查看调整的结果

当调整比较理想的状态之后，记录对应的值：

    ```yaml
    tf_x: -2.42
    tf_y: -0.65
    tf_rot_z: 0.116
    ```

关闭所有终端后，将其更新到配置文件中

    ```bash
    $ ssh mind@192.168.1.101
    ## 打开要修改的文件
    $ vim .config/autostart/nav_parameters.yaml
    ```

    ```yaml
    ## adjust parameters for alignment of map and gnss 
    pose_estimation:
        pose_source: lidar ## gps or lidar
        tf_x: -2.42
        tf_y: -0.65
        tf_rot_z: 0.116
    ```
    修改完成并经仔细检查后保存退出。

**恢复到导航模式**

```bash
$ ssh mind@192.168.1.101 
## 打开配置文件，
$ vim .config/autostart/auto_startup.sh
## 将debug_state修改为false，
## 且record_data修改为false,
## align_mapgnss保持为false，保存后退出
sudo reboot
```