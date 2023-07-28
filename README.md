```bash
sudo apt update
sudo apt-get install openssh-server
sudo systemctl enable ssh
```
Remember to turn off the sleep mode and turn on the automatic login.
Now you can use ssh from your PC

```bash
sudo apt-get install git python3-pip python3-schedule raspi-config wget pyserial curl -y
Then open the tool using
sudo raspi-config
```
Once the tool screen is opened, go to Interface Options and then enable "Legacy camera option".

#### Enable Camera

```bash
sudo usermod -aG video USER-NAME
sudo nano boot/firmware/config.txt
start_x = 1 # Add this line at the bottom

# From command line, you could use vcgencmd utility as shown below:
vcgencmd get_camera
supported=1 detected=1, libcamera interfaces=0

# sudo apt install v4l2-ctl
# ls /dev/video*

# When no camera detected the output would be:
vcgencmd get_camera
supported=0 detected=0
```

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/TPODAvia/Autonomous-Tractor.git

chmod +x ROS2-installation/ROS2_server.sh
sudo ./ROS2-installation/ROS2_server.sh

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```
#### Encrease swap file:

See this instructions for the details https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-22-04

### Add watchdog

Activate the hardware watchdog on your Raspberry Pi. With the command `sudo nano /boot/config.txt`, add the following line to enable the watchdog:

```bash
dtparam=watchdog=on
```

Reboot your Raspberry Pi. After the reboot, you can check if the watchdog is enabled by running the command `ls -al /dev/watchdog*`. You should see output similar to this:

```bash
crw------- 1 root root  10, 130 May 19 07:09 /dev/watchdog
crw------- 1 root root 253,   0 May 19 07:09 /dev/watchdog0
```
This indicates that the watchdog is enabled.

Install the watchdog software package with the command `sudo apt-get install watchdog`.

Configure the watchdog package. Edit the configuration file with the command `sudo nano /etc/watchdog.conf` and uncomment or add the following lines:

```bash
max-load-1 = 24
min-memory = 1
watchdog-device = /dev/watchdog
watchdog-timeout=15
```

Start the watchdog service with the command `sudo systemctl start watchdog` and enable it to start at boot with the command `sudo systemctl enable watchdog`.

Verify that the watchdog service is running with the command `sudo systemctl status watchdog`. If it's running correctly, you should see output indicating that the service is active.

### Install ORB-SLAM3

```bash

sudo apt-get install libboost-all-dev libboost-dev libssl-dev libpython2.7-dev libeigen3-dev 
# sudo apt-get install  libunwind-dev
# sudo apt-get install libdc1394-dev
sudo apt install ros-humble-vision-opencv && sudo apt install ros-humble-message-filters

cd
git clone https://github.com/TPODAvia/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
cd ORB_SLAM3
chmod +x build.sh
./build.sh
https://github.com/astronaut71/orb_slam3_ros2

```
### Install OpenCV

```bash
# wget https://github.com/Qengineering/Install-OpenCV-Raspberry-Pi-32-bits/raw/main/OpenCV-4-5-5.sh
cd ~/colcon_ws/src
sudo chmod 755 /ROS2-installation/OpenCV-4-5-5.sh
./ROS2-installation/OpenCV-4-5-5.sh
```
### Compile ROS2 project

```bash
cd ~/colcon_ws/src

git clone https://github.com/TPODAvia/orb_slam3_ros2.git orbslam3_ros2

git clone https://github.com/cra-ros-pkg/robot_localization.git
cd robot_localization
git checkout humble-devel
cd ..

git clone https://github.com/RobotWebTools/web_video_server.git
cd web_video_server
git checkout ros2
cd ..

git clone https://github.com/ros2/teleop_twist_keyboard.git

cd ~/colcon_ws
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select orbslam3
colcon build
```

#### Working with GPS and IMU
```bash
https://github.com/jefmenegazzo/mpu-i2c-drivers-python
sudo apt-get install i2c-tools python3-smbus
# pip install smbus2 ahrs
sudo adduser ubuntu i2c
i2cdetect -y -r 1
sudo i2cget -y 1 0x68 0x75

sudo apt-get install libi2c-dev
git clone https://github.com/hiwad-aziz/ros2_mpu6050_driver.git
sudo chmod 777 /dev/ttyUSB0
git clone https://github.com/orascheg/ros2GPSx.git
```

### Install path planner
```bash
git clone https://github.com/nobleo/full_coverage_path_planner.git
cd full_coverage_path_planner
git checkout ros2
rosdep install --from-paths src --ignore-src -r -y
```
### How to use

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
cd ~/colcon_ws
ros2 launch my_cpp_py_pkg camera.launch.py
ros2 launch my_cpp_py_pkg web_control.launch.py
```