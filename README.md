 One of the key challenges in mobile robots is the perception problem, but what is perception? Perception
 consists of making sense of the unstructured real world, in other words converting the real world into a
 more manageable version of it so that robots can work more efficiently with it. Remember that robots
 have a limited computation power, thus selecting what is important becomes a challenge and it is mostly
 driven by the application.
 You have seen some of the most popular sensors used for robot perception in previous laboratories, such
 as ultrasonic sensors and LIDARs; however, you have not used one of the most important ones: the camera.
 In this laboratory, you will learn to use several libraries for image processing such as OpenCV.Although
 it does not have native support to work with ROS, cv bridge will help us convert CV image objects into
 ROS messages that can be exchanged between nodes in a ROS application.
 As part of this laboratory, you will utilize some algorithms for the robot to detect a moving red ball
 using the camera and to follow the red ball. In addition to OpenCV, you will have to work on the camera
 calibration, which will provide us with the required information to do other tasks such as marker detection
 and localization. In this lab you will learn how to use the ROS fiducial markers library, which provides
 with the tools to create, identify and locate tags with respect to the camera frame. This will allow the
 system to have a position reference system on top of which we can perform some control tasks.
