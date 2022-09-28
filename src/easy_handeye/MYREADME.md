# IRM Lab Version of **EASY_HANDEYE**
*modified by jyp, yxj*


### Log
* **0421**
    * 绘制了3D打印的标准连接件(孔径6.2mm，适应M6螺丝；直径50mm的圆周上阵列$\times{4}$)，打印文件在yxj的U盘，控制Franka的Win系统可能找到副本；
    * 完成了Franka和Basler相机的Eye-to-Hand标定
    * 最关键：launch文件`/home/roboticslab/yxj/ws_franka/src/easy_handeye/easy_handeye/launch/panda_basler_eyeonbase.launch`，负责整个Eye-to-Hand标定的流程。该文件自上而下分为4部分，Moveit控制机械臂(随型号而异)部分/相机启动部分/Aruco码识别部分/EASY_HANDEYE的calibrate部分launch文件
    * 需要修改的关键参数：
      | parameter | location | definition |
      | :----: | :----: | :----: |
      | load_gripper | line 8 | 安装标准件后，设为False，否则报错夹爪不存在 |
      | markerId/markerSize | line 21/22 | Aruco码的编号及尺寸 |
      | marker_frame | line23 | Aruco码的坐标系名称，全文件保持一致 |
      | ref_frame | line24 | Aruco码的位姿将以相对此坐标系的数据被publish出来，建议不要留空 |
      | camera_frame | line26 | 一般选相机光心坐标系的名称 |
      | camera_image_topic | line27 | 相机发布image_raw图像的topic |
      | camera_image_info | line28 | 相机发布info的topic |
      | robot_base_frame | line51 | 机器人基坐标系$B$ |
      | robot_effector_frame | line52 | 机器人末端执行器坐标系 |
      | tracking_base_frame | line55 | 一般选相机坐标系`marker_frame`或`ref_frame`，二者通过我们发布的静态坐标变换保持一致 |
      | tracking_marker_frame | line56 | 与`marker_frame`保持一致 |
      注：在launch文件中添加以下代码，手动发布tf变换，使`marker_frame`或`ref_frame`保持一致
      ```
        <node pkg="tf2_ros" type="static_transform_publisher" name="to_publish_a_camera_link_frame" args="0 0 0 0 0 0 1  camera_link camera_color_frame" />
      ```
    * 打开机械臂和相机
      注：相机易发热损坏，长时间不用请务必拔掉USB
    * 运行launch文件
      ```
        roslaunch easy_handeye panda_basler_eyeonbase.launch
      ```
    * 标定文件存放在`~/.ros/easy_handeye/panda_eob_calib_eye_on_base_0430.yaml`，（该文件标定的是相机被提高过的位置，非常准确）同目录下`backup`存放的是曾经标定过的数据

    * ***BUG排除***
      1. Python2.7环境下可能报错`cv2: 'module' object has no attribute'CALIB_HAND_EYE_TSAI'`  
         原因：Opencv版本问题  
         解决：  
         ```
            pip install opencv-python==4.2.0.32
         ```
      2. aruco_marker_frame到ref_frame间坐标变换的translation部分出现`nan`，且tf报错  
         原因：相机未标定，检查terminal发现`The camera is not calibrated`提示  
         解决：使用棋盘格标定相机  
         注意：@TODO 标定`.yaml`文件存放位置  
      3. (未解决)多次标定后结果不稳定，x坐标方差较大，旋转四元数方差较大  
         猜测：某些标定版Pose会导致矩阵运算时出现接近奇异的情形，得到了不稳定的结果  
               标定板接近水平，z轴几乎竖直向上，Aruco码在此位置的形状不容易发生变化，z轴方向不稳定，导致记录了几组偏差较大的数据(也可能是环境光干扰的原因)  
               Franka关节限位导致某些Pose下关节角不正确

    * 参考
      * EASY_HANDEYE官方README.md
      * lkc在云盘上的文档：(链接)@TODO
