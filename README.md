# ROBOTIS-OP3  
## motion following demo using openpose_ros  
1. openpose  

2. openpose_ros  
  
3. robotis_op3_following_motion  
  
## object tracking demo using YOLO  
1. [YOLO](https://pjreddie.com/darknet/yolo/)  
  Real-Time Object Detection, It is 'You only look once(YOLO).'  
  
2. [YOLO-ROS](https://github.com/leggedrobotics/darknet_ros)  
  This is a ROS package developed for object detection in camera images. You only look once (YOLO) is a state-of-the-art, real-time object detection system. In the following ROS package you are able to use YOLO (V3) on GPU and CPU.  
  
3. object_tracking_yolo  
  - How to run  
    - ROBOTIS-OP3  
      make new launch file like below and run the launch file  
  
      ```
      <launch>
        <!-- robotis op3 manager -->
        <include file="$(find op3_manager)/launch/op3_manager.launch"/>

        <!-- UVC camera -->
        <node pkg="usb_cam" type="usb_cam_node" name="usb_cam_node" output="screen">
          <param name="video_device" type="string" value="/dev/video0" />
          <param name="image_width" type="int" value="1280" />
          <param name="image_height" type="int" value="720" />
          <param name="framerate " type="int" value="30" />
          <param name="camera_frame_id" type="string" value="cam_link" />
          <param name="camera_name" type="string" value="camera" />
        </node>

        <!-- camera setting tool -->
        <include file="$(find op3_camera_setting_tool)/launch/op3_camera_setting_tool.launch" />

        <!-- web setting -->
        <include file="$(find op3_web_setting_tool)/launch/web_setting_server.launch" />
      </launch>

      ```  
    - Laptop(GPU)  
      - execute yolo-ros  
      ```  
      $ roslaunch object_tracking_yolo yolo_v3.launch
      ```
      - execute `object tracking demo`  
      ```
      $ roslaunch object_tracking_yolo object_tracking.launch
      ```

  - Operation method  
    - Start/Stop : START Button  
    - Go initial : MODE Button  
    - Changing the target : USER Button  
 
  - Parameter setting  
  - Performance  
    - ROBOTIS-OP3, CPU, One image : 62sec  
    - Laptop(i7-6700HQ, GTX970M), GPU, One image : 0.1sec (10~11 fps)  

