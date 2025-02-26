# Office-Assitant-Robot
to run the yolo detecter, first copy folder "yolo_detecter" to /tiago_public_ws/src
make sure all py files have permission to execute

Then copy tiago_gazebo.launch.py to /tiago_public_ws/src/tiago_simulation/tiago_gazebo/launch. Replace the original launch file

install requirements:  
`pip install ultralytics opencv-python numpy`  

first time run:  
`cd ~/tiango_public_ws`  
`colcon build --packages-select yolo_detector`  

start detecter node:  
`cd ~/tiango_public_ws/yolo_detecter/yolo_detecter`  
`python3 yolo_detecter.py`  