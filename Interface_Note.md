# 巡检车中间层与底盘通信接口设计
****
## 1. 接口设计说明
&emsp;&emsp;巡检车端的软件系统目前分为三层，具体分为 **功能实现层**、**中间通信层**、**任务调度层**。其中功能实现层包含高精度地图构建、3D点云融合定位、路径规划、点云聚类识别、避障、运动规划、控制执行等模块，是巡检车智能化运动控制的底层实现。任务调度层用于实现用户交互，包括定制化巡检车的任务执行方式、显示巡检的各种状态信息等。中间通信层则主要是对功能实现层的接口进行包装，并按照服务机器人协议同任务调度层通信，同时实现包括建立数据库对巡检车配置信息进行存储、地图上传等功能。

#### 1.1 适用范围
&emsp;&emsp;该接口适用于巡检车 ***功能实现层*** 与 ***通信中间层*** 之间的通信连接。

#### 1.2 通信方式
&emsp;&emsp;采用**ROS**(一种基于消息传递通信的分布式多进程通信框架)的Topic通信机制，通过建立Node节点进行数据传输。

#### 1.3 平台依赖
&emsp;&emsp;**硬件平台** :  Jetson AGX Xavier 

&emsp;&emsp;**操作系统** :  Ubuntu 18.04

&emsp;&emsp;**环境依赖** :  ROS(Melodic) / Autoware-msgs / Scout-msgs

#### 1.4 接口类别
&emsp;&emsp;主要包括**两类**接口:   底盘(包括传感器)运行**状态**信息和底盘(包括传感器)运动**控制**信息。

****
## 2. 主要通信接口设计
### 2.1 状态信息
#### 2.1.1 底盘状态信息

**Topic Name** : `/scout_status`

**Msg Type** : `scout_msgs/ScoutStatus`

**Msg Content** :  `时间戳` `线速度` `角速度` `系统状态` `控制模式(遥控/自动控制)`  `底盘错误代码` `电量` `电流` `指示灯状态` `软/硬急停`
```python
std_msgs/Header header        ## 时间戳
  uint32 seq
  time stamp
  string frame_id
float64 linear_velocity        ## 线速度
float64 angular_velocity        ## 角速度
uint8 base_state        ## 底盘状态 系统正常-0x00 硬急停-0x02 系统异常-0x02
uint8 control_mode         ## 控制模式  0x00 -- 遥控模式(软急停)  0x02 -- 自动模式 见
uint16 fault_code        ## 底盘错误代码，见 Note 1
float64 battery_voltage        ## 电量信息 Percent = (29.4 - battery_voltage) / 8.4
scout_msgs/ScoutMotorState[4] motor_states   ## 电流信息 
  float64 current
  float64 rpm
  float64 temperature
bool light_control_enabled
scout_msgs/ScoutLightState front_light_state  ## 前指示灯状态
  uint8 mode
  uint8 custom_value
scout_msgs/ScoutLightState rear_light_state  ## 后指示灯状态
  uint8 mode
  uint8 custom_value
```
**Note 1** :  `falult_code`对照表
| 数据位 | 含义 |
---|---|---
| bit[15] - bit [12] | Reserved |
| bit[11] | 电池欠压警告(0: 无警告 1: 警告) 报警电压22.5V |
| bit[10] | 电机过流警告(0: 无警告 1: 警告) 报警电流15A |
| bit[9]  | 电机驱动过温警告(0: 无警告 1: 警告) 报警温度55℃ |
| bit[8]  | 通信控制指令校验错误警告(0: 无警告 1: 警告) |
| bit[7]  | 电机过流保护 (0: 无保护 1: 保护) 电流有效值20A |
| bit[6]  | 电机驱动过温保护(0: 保护 1: 保护) 温度限制65℃ |
| bit[5]  | 电机4通讯故障(0: 无故障 1: 故障) |
| bit[4]  | 电机3通讯故障(0: 无故障 1: 故障) |
| bit[3]  | 电机2通讯故障(0: 无故障 1: 故障) |
| bit[2]  | 电机1通讯故障(0: 无故障 1: 故障) |
| bit[1]  | 电池欠压故障(0: 无故障 1: 故障) 保护电压22.5V|

**Note 2**: 软/硬急停状态 
| 急停 | 标志值 |
---|---|---
| 硬急停 | base_state == 0x02 |
| 软急停 | control_mode  == 0x00 |

#### 2.1.2 全局路径规划状态信息
**Topic Name** : `/global_path_state`

**Msg Type** : `std_msgs/Bool`

**Msg Content** :   `全局路径规划状态`

```
uint8 data
```


#### 2.1.3 全局路径点信息
**Topic Name** : `/lane_waypoints_array`

**Msg Type** : `autoware_msgs/LaneArray`

**Msg Content** :   `路径点`

```python
int32 id
Lane[] lanes
  Header header
  int32 increment
  int32 lane_id
  Waypoint[] waypoints  ## 路径点

  uint32 lane_index
  float32 cost
  float32 closest_object_distance
  float32 closest_object_velocity
  bool is_blocked
```

#### 2.1.4 运动状态信息

**Topic Name** : `/current_behavior`

**Msg Type** : `geometry_msgs/TwistStamped`

**Msg Content** :   `运动状态等信息`

```shell
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x  ## 重新规划
    float64 y  ## 跟踪距离
    float64 z  ## 跟踪速度
  geometry_msgs/Vector3 angular
    float64 x  ## 路径点标识号
    float64 y  ## 当前状态(STATE_TYPE)
    float64 z  ## 路线标识号
```

其中，当前状态`STATE_TYPE`取值如下，任务完成时取值`FINISH_STATE(13)`

```C++
enum STATE_TYPE {INITIAL_STATE, WAITING_STATE, FORWARD_STATE, STOPPING_STATE, EMERGENCY_STATE,
                  TRAFFIC_LIGHT_STOP_STATE,TRAFFIC_LIGHT_WAIT_STATE, STOP_SIGN_STOP_STATE, 
                  STOP_SIGN_WAIT_STATE, FOLLOW_STATE, LANE_CHANGE_STATE, OBSTACLE_AVOIDANCE_STATE, 
                  GOAL_STATE, FINISH_STATE, YIELDING_STATE, BRANCH_LEFT_STATE, BRANCH_RIGHT_STATE};
```

#### 2.1.5 巡检车当前位置
**Topic Name** : `/curent_pose`

**Msg Type** : `geometry_msgs/PoseStamped`

**Msg Content** :   `位置信息`

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

### 2.2 控制指令
#### 2.2.1 指定目标点指令
**Topic Name** : `/target_goal`

**Msg Type** : `geometry_msgs/PoseStamped`

**Msg Content** :  
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

#### 2.2.2 取消目标指令 
**Topic Name** : `/cancel_goal`

**Msg Type** : `geometry_msgs/PoseStamped`

**Msg Content** :  

#### 2.2.3 相机云台控制
**Topic Name** : `/camera/cam_movectr`

**Msg Type** : `std_msgs/UInt8`

**Msg Content** :   `运动指令`    
```
uint8 data
```

|值 | 含义 |
---|---|---
| 0x01 | 相机云台右转单位弧度 |
| 0x02 | 相机云台左转单位弧度 |
| 0x03 | 相机云台向上转单位弧度 |
| 0x04 | 相机云台向下转单位弧度 |
| 0xa1 | 相机云台水平自动旋转 |
| 0xa2 | 相机云台回到初始位置 |
| 0xff   | 相机云台停止运动 |


#### 2.2.4 软急停控制指令
**Topic Name** : `/emergency_state`

**Msg Type** : `std_msgs/Bool`

**Msg Content** :   `急停`
```
uint8 data
```
|值 | 含义 |
---|---|---
| False | 取消软急停 |
| True  | 使能软急停 |

