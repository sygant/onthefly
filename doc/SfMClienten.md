## On-the-Fly SfM Image Transmission System Instructions

This system represents an integrated platform encompassing image data acquisition, wireless transmission, and display capabilities. In conjunction with the On the fly System, it serves as an interactive gateway on mobile devices. The system comprises two primary components: an Android application and a server-side infrastructure. Through the utilization of the efficient WebSocket communication protocol, it facilitates real-time, stable, and secure transmission of imagery and other pertinent data, making it a valuable asset for academic research and applications.

### Project Homepage

On-the-Fly SfM (source code)：

<https://github.com/RayShark0605/On_the_fly_SfM>

On-the-Fly SfMv2 (project website)：

<https://yifeiyu225.github.io/on-the-flySfMv2.github.io>



### Usage



#### Instructions for mobile app

##### 1.Download the latest version of the client app

<https://github.com/sygant/onthefly/releases/download/v1.0.0/onthefly.apk>



##### 2.Authorization for the installed app

An authorization screen will appear upon the first launch of the app. Accessing the camera and external storage files are considered sensitive permissions in the Android system. However, these permissions are necessary for the operation of this system and please grant the permissions.



##### 3.Set the server address and port

After obtaining application permissions to enter the home screen, you should first set the address and connect to the server.

The home screen is shown as below：

<img src="_static\images\1714048336125.png" alt="1714048336125" style="zoom: 25%;" />



Then click the setting button to enter the setting screen：

<img src="_static\images\1714048336121.png" alt="1714048336121" style="zoom:25%;" />

Enter the server address and port number in the Server ip field, the format is **http://[ip]:[port]**



##### 4.Restart the application, and enter the video screen

<img src="_static\images\1714048336114.png" alt="1714048336114" style="zoom:25%;" /><img src="_static\images\1714048336105.png" alt="1714048336105" style="zoom:25%;" />

In the upper right corner of the application page, the gray indicator indicates that the client is not connected to the server, and the green indicator indicates that the connection is normal and the photos can be transferred.

 

- Capture button: Take photos and transfer them to the server

- More (down arrow) button: Display the console information, the storage path of the photo, the connection details with the server, the image transfer progress can be viewed in this console.



##### 5.Attention

Every time you change the server ip address and port in application Settings, you must restart the application for the change to take effect.



#### Instructions for server side settings

##### 1.Download the latest server

<https://github.com/sygant/onthefly/releases/download/v1.0.0/OntheflyServer.rar>

 

##### 2.Decompress the directory and double-click the bat file

If the following output is displayed, the server is running successfully:

<img src="_static\images\20240426115953047.png" alt="20240426115953047" style="zoom:100%;" />

##### 3.Attentions

- The server uses the port 9001 by default. Ensure that port 9001 is not used before running.

- The upload file address defaults to /upload, and can be changed in the bat file to customize the upload path with the --uploadfolder parameter.

- Server file path must be in English and cannot contain Spaces.

- The server can connect to multiple clients, and all images are transferred to the upload folder.