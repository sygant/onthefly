## On the fly SfM 实时图像传输系统介绍

本系统是一款集影像数据采集、无线传输、显示于一体的综合性系统，与Onthefly软件搭配使用，作为其在移动端的交互入口。该系统由安卓端app和服务端两部分构成，通过高效的WebSocket通信协议，实现实时、稳定、安全的图像以及其它数据传输任务。

### 项目主页

On-the-Fly SfM (source code)：

<a href="https://github.com/RayShark0605/On_the_fly_SfM" title="link">On the fly SfM</a>

On-the-Fly SfMv2 (project website)：

<a href="https://yifeiyu225.github.io/on-the-flySfMv2.github.io " title="link">On the fly SfMv2</a>



### 使用说明



#### 手机端app使用说明

##### 1.下载最新版的客户端

//TODO 下载链接link



##### 2.安装应用授权

初次使用应用，会出现授权界面，在安卓系统中访问摄像头和应用外的文件属于敏感权限，这些权限是运行本系统所必需的权限，不会涉及应用安全问题，请放心授权。



##### 3.设置服务端地址

应用主界面如图：

<img src="_static\images\1714048336125.png" alt="1714048336125" style="zoom: 25%;" />

然后点击setting按钮进入设置界面：

<img src="_static\images\1714048336121.png" alt="1714048336121" style="zoom:25%;" />

Server ip栏填入服务端地址和端口号，格式为**http://[ip]:[port]**



##### 4.重启应用，进入拍摄页面

<img src="_static\images\1714048336114.png" alt="1714048336114" style="zoom:25%;" /><img src="_static\images\1714048336105.png" alt="1714048336105" style="zoom:25%;" />

观察应用界面右上角，连接指示灯为灰色表示未与服务器连接，绿色指示灯表示与服务器连接正常，可以传输拍摄的照片。

 

- Capture按钮：拍摄照片并传输到服务器

- 更多（下箭头）按钮：显示控制台信息，拍摄照片的存储路径、与服务器连接详细信息、图像传输进度都可以查看。



##### 5.注意事项

每次在应用设置中更改服务器ip地址后均要重启应用才能生效。



#### 服务端配置说明

##### 1.下载最新版的服务端

//TODO下载链接 link

 

##### 2.解压目录，双击bat文件即可

出现如下输出则表示服务端运行成功：

<img src="_static\images\20240426115953047.png" alt="20240426115953047" style="zoom:100%;" />

##### 3.注意事项

- 服务端默认使用9001端口，请在运行前务必保证该端口未被占用

- 上传文件地址默认为/upload，可以在脚本文件中更改--uploadfolder参数来自定义上传路径

- 服务端文件路径必须是全英文路径，且不能包含空格

- 服务端可连接多个客户端，所有影像均被传输至upload文件夹下