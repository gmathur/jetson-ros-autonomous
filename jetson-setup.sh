echo "Add passwordless sudo by adding this to visudo"
echo "nvidia  ALL=(ALL) NOPASSWD: ALL"
read -p "Press enter to continue"

git config --global user.email "g@gmathur.com"
git config --global user.name "Gaurav Mathur"
cp .vimrc ~/
cp -R .vim/ ~/

echo "SSH setup"
sudo apt-get -y install openssh-server curl htop
sudo service ssh start
mkdir ~/.ssh
ssh-keygen -f ~/.ssh/id_rsa -t rsa -N ''
curl -u "gmathur" --data "{\"title\":\"jetson-key\",\"key\":\"$(cat ~/.ssh/id_rsa.pub)\"}" https://api.github.com/user/keys

echo "ROS setup"
./modules_jetson/installROSTX2/installROS.sh -p ros-kinetic-desktop-full
./modules_jetson/installROSTX2/setupCatkinWorkspace.sh
sudo apt-get -y install ros-kinetic-razor-imu-9dof python-visual ros-kinetic-serial \
     ros-kinetic-ackermann-msgs ros-kinetic-joy ros-kinetic-gmapping ros-kinetic-rtabmap-ros \
     jstest* joystick xboxdrv vim-gnome ros-kinetic-xacro ros-kinetic-amcl \
     ros-kinetic-robot-state-publisher ros-kinetic-rgbd-launch

echo "Udev rules setup"
sudo usermod -a -G dialout nvidia
sudo usermod -a -G video nvidia
sudo usermod -a -G audio nvidia
cd modules_jetson/installRACECARUdev; ./installRACECARUdev.sh
cd ../..
sudo cp ./razor_imu_m0_driver/razor_imu_m0_driver.rules /etc/udev/rules.d
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Realsense setup"
cd modules_jetson/buildLibrealsense2TX
./buildPatchedKernel.sh 
./installLibrealsense.sh

source ~/catkin_ws/devel/setup.bash
