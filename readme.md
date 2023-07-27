https://github.com/jefmenegazzo/mpu-i2c-drivers-python

sudo apt-get install -y i2c-tools python3-smbus
sudo adduser ubuntu i2c

   pip install smbus2
pip install smbus2 ahrs

git clone https://github.com/cra-ros-pkg/robot_localization.git
git checkout humble-devel
rosdep install --from-paths src --ignore-src -r -y

sudo apt install ros-humble-vision-opencv && sudo apt install ros-humble-message-filters

https://github.com/astronaut71/orb_slam3_ros2
