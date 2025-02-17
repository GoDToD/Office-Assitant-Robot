# Office-Assitant-Robot
to run the yolo detecter, first copy folder "yolo_detecter" to /tiango_public_ws/src
make sure all py files have permission to execute

install requirements:
pip install ultralytics opencv-python numpy


first time run:
cd ~/tiango_public_ws\n
colcon build --packages-select yolo_detector

start detecter node:
cd ~/tiango_public_ws/yolo_detecter/yolo_detecter\n
python3 yolo_detecter.py