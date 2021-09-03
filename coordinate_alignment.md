## GPS coordinate alignment

选择GPS参考点主要是让巡检车base_link frame和gps frame对齐，但是由于地球坐标系的投影偏差，在距离参考点越远的地方偏差越大，所以最好将参考点选在所构建地图的中央来平衡偏差。
在巡检车中，为了方便可以设置地图作为参考点。

### 准备工作

* `ssh`登录到巡检车，屏蔽`world`到`map`之间的`tf`转换，具体操作如下：

```shell
## 登录 导航工控机
## user: mind
## password: mind
$ ssh mind@192.168.1.101 
$ rosed car_bringup startup_map_loader.launch
```

打开文件后，找到对应行，按如下方式进行屏蔽:

```xml

<!-- node pkg="tf" type="static_transform_publisher" name="world_to_map" args="0.0 0.0 0.0 1.74 0.0 0.0 /world /map 40" / -->

```
保存后退出，`ctrl + d`关闭终端

* 建立安装有`ros`的`ubuntu`笔记本和巡检车工控机之间的通信(在连到巡检车wifi的情况下操作)，打开笔记本的终端(不要ssh登录)，进行如下操作:

```shell
$ ifconfig ## 查看笔记本的IP地址
$ echo "export ROS_MASTER_URI=http://192.168.1.101:11311 " >> ~/.bashrc
## 将192.168.1.103修改为笔记本的IP
$ echo "export ROS_IP=192.168.1.103 " >> ~/.bashrc
```

关闭终端重新打开，进行如下配置验证，如果输出正常，则设置成功：

```shell
rosnode list
```

退出终端

* 修改自启文件，设置GPS模块自启:

```shell
## 登录 导航工控机
## user: mind
## password: mind
$ ssh mind@192.168.1.101 
## 打开自启文件，
$ vim .config/autostart/auto_startup.sh
## 将run_GPS修改为true，保存后退出
```

### 1 选择GPS参考点在

将巡检车移动到室外地图原点，并登录到`GPS`模块的后台(address: `192.168.10.100`)，等到`GPS`的`position type`变为`rtk fixed`时(条件良好的情况下，通常需要5分钟左右),记录此时的`GPS`坐标(单位: 度分秒)作为参考点，并进行弧度转换，示例如下：

```shell

## example
## 3224.7893249,N,11933.6781869,E  <<<<<=转换为=>>>>> 32.413155415  119.561303115    
## 3224.7899868,N,11933.6784922,E  <<<<<=转换为=>>>>> 32.4131664467 119.5613082033

```

*Note :
* 参考点比较重要，一定要使用GPS稳定后的数据作为参考点(RTK fixed) 
* 在整个设置参考点，坐标对齐的过程中，不要移动巡检车的位置


记录转换为弧度单位的`GPS`坐标，并将其设置到巡检车的配置文件中:

```shell
## 登录 导航工控机
## user: mind
## password: mind
$ ssh mind@192.168.1.101 
$ rosed car_bringup startup_map_loader.launch
```

根据将记录的参考点坐标，修改`origin_lat`和`origin_lon`的`value`， 修改位置如下：

```xml
  <node name="nmea2tfpose" pkg="gnss_localizer" type="nmea2tfpose_mpe" >
    <param name="plane" value="20" /> 
    <param name="origin_lat" value="32.400122947" />
    <param name="origin_lon" value="119.561132022" />
    <param name="meter_per_degree_lat" value="110893.81565767189" />
    <param name="meter_per_degree_lon" value="94080.45099898388" />
  </node>
```

保存后退出，并重启导航工控机:

```shell
sudo reboot
```

### 2 坐标对齐

重启导航工控机5分钟后，巡检车完成自我定位，并查看`GPS`模块后台，`position type`在`rtk fixed`状态，此时可以进行坐标对齐，具体操作如下：

`ubuntu`笔记本打开终端1(此时笔记本已经和导航工控机建立通信)

```
rosrun rviz rviz
```

打开可视化界面后，将`Global Options`的`Fixed Frame`设置为`map`, 并打开`tf`插件，重点关注`base_link frame`和`gps frame`，然后进行对齐操作，主要是将这两个坐标系对齐。

打开新终端，通过不断修改如下指令中的参数进行对齐（`ctrl+c` 终止，然后修改参数尝试），主要是`yaw`, `x`, `y`：

```shell
## 登录 导航工控机
## user: mind
## password: mind
$ ssh mind@192.168.1.101 
rosrun tf static_transform_publisher 0.0 0.0 0.0 1.74 0.0 0.0 /world /map 40
## 修改如上的6个参数，分别是x,y,z,yaw,pitch,roll
```

不断观察`rviz`两个坐标系`base_link frame`和`gps frame`是否对齐来反复调整参数，对齐后还要移动巡检车观察`rviz`中`base_link frame`和`gps frame`是否同步移动(可能出现向相反方向移动的情况)。

在当对齐，并满足同步移动后，记录该 `6` 个参数。

然后关闭终端

### 3 参数配置

* 解除屏蔽`world`到`map`之间的`tf`转换，具体操作如下：

```shell
## 登录 导航工控机
## user: mind
## password: mind
$ ssh mind@192.168.1.101 
$ rosed car_bringup startup_map_loader.launch
```

打开文件后，找到对应行，解除屏蔽，并将记录的 `6` 个参数更新到其中(注意对应关系):

```xml

<node pkg="tf" type="static_transform_publisher" name="world_to_map" args="0.0 0.0 0.0 1.74 0.0 0.0 /world /map 40" />

```
保存后退出，`ctrl + d`关闭终端


* 配置参数，将用`GPS`信息用于定位

```shell
## 登录 导航工控机
## user: mind
## password: mind
$ ssh mind@192.168.1.101 
$ rosed car_bringup startup_matching.launch
```

将其中的`init_pos_gnss`和`use_gnss`的`value`由 `0` 修改为 `1` ，即如下两行：
```xml
...

    <param name="init_pos_gnss" value="0" /> <!--weather -->

...

    <param name="use_gnss" value="0" />  <!-- 和下面config中的init_pos_gnss一个作用-->

...
```
保存退出后，关闭终端

### 4 完成配置，进行验证

进行如上操作之后，可以整机重启，等待`GPS`模块`fixed`之后，巡检车即可完成任意位置定位，意即：
* 不一定在原点开机，即可以实现定位
* 从室内开到室外，不需要重启即可完成定位


*Note :
* `GPS`模块实现`rtk fixed`的时间所处的环境相关，所以开阔地带能很快实现定位，非开阔地带则有可能定位失败。因此，参考点一般要选在开阔地带，初始定位一般也最好将车移动到开阔地带。

