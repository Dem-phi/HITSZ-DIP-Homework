### DIP Task

#### calibrate the camera 

use the camera_calibration to calibrate the camera, and get the camera internal parameters


```bash

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 --no-service-check right:=/camera/fisheye1/image_raw left:=/camera/fisheye2/image_raw right_camera:=/camera/fisheye1 left_camera:=/camera/fisheye2

```

#### planning
Step1-轨迹跟踪


Step2-速度规划+planning
Bezier曲线

publish tf 
```bash
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map world 100
```                     