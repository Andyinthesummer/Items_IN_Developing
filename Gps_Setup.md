### GPS Setup

在进行gps-rtk模块配置之前，需要自建或购买基站服务。目前国内使用[千寻位置](https://www.qxwz.com/)，新加坡使用[SiReNt](https://app1.sla.gov.sg/SIRENT/Page/Services)提供的虚拟基站服务。

主要原理：利用Ntrip帐号将本地gps信息(NMEA)发送给虚拟基站，并从虚拟基站接收RTCM校准数据，GPS模块经过rtk运算，获取精度较高的gps数据，最后通过USB端口将gps信息传递给工控机使用。

*Notes:*
* 目前主GNSS系统设置为gps(支持GLONASS和BeiDou),每个天线至少能连上4颗gps卫星时，模块才能输出fixed精度和可信度高的定位结果

#### Trimble MB2 setup

```yaml
IP: 192.168.10.100
## IP: 192.168.1.100
login: admin
password: Gison2015Gison
```

**MB2 overall Infomation**   

![](images/mb2_info.png)    

**Sensors/Antennas Setup**    

![](images/ants_setup.png)    

**Rover Setup**

![](images/rover_setup.png)    

**Heading Setup**

![](images/heading_setup.png)    

**I/Os Setup**

![](images/IO_setup.png)    

*Software Setup*

利用`gnss_localizer`将经纬度转换为`UTM`坐标系，注意`UTM`是分区的，不同区域单位经纬度对应不同标准距离(不过`gnss_localizer`在设置好参考坐标之后能够直接根据分区转换)，设置`plane`和设置参考`GPS`坐标的结果是一样的，属于重映射函数

*Setup Ref. Point*

```
3224.7893249,N,11933.6781869,E  <<<<<===>>>>> 32.413155415  119.561303115    
3224.7899868,N,11933.6784922,E  <<<<<===>>>>> 32.4131664467 119.5613082033
```
设置参考点作为`world`坐标系，利用`static_transform_publisher`校准`world`和`map`之间的坐标转换。
