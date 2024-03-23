# change source
sudo mv /etc/apt/sources.list /etc/apt/sources.list.bake
sudo sh -c "echo '# 默认注释了源码镜像以提高 apt update 速度，如有需要可自行取消注释
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-updates main restricted universe multiverse
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse
# deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-backports main restricted universe multiverse

deb http://security.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse
# deb-src http://security.ubuntu.com/ubuntu/ jammy-security main restricted universe multiverse

# 预发布软件源，不建议启用
# deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-proposed main restricted universe multiverse
# # deb-src https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ jammy-proposed main restricted universe multiverse' > /etc/apt/sources.list"

sudo apt update
sudo apt upgrade -y
sudo apt install gedit gcc-12 g++-12 cutecom libopencv-dev libopencv-contrib-dev libceres-dev libeigen3-dev build-essential gdb git cmake htop curl gnupg2 wget -y

# install ros
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
# 手动模拟 rosdep init
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
sudo curl -o /etc/ros/rosdep/sources.list.d/20-default.list https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
# 为 rosdep update 换源
export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml
sudo apt install python3-rosdep2 -y
rosdep update
# 每次 rosdep update 之前，均需要增加该环境变量
# 为了持久化该设定，可以将其写入 .bashrc 中，例如
echo 'export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml' >> ~/.bashrc
echo 'source ~/rmvision/install/setup.bash' >> ~/.bashrc

# add some convinient alias
echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=Yes'
alias mdc='mkdir build && cd build'
alias cl='cd ..'
alias rd='rm -r'
alias md='mkdir'
" >> ~/.bashrc

# sudo dpkg -i MvCamCtrlSDK_Runtime-4.3.0_x86_64_20231225 -y
sudo dpkg -i MVS-2.1.2_x86_64_20231225.deb 
sudo dpkg -i code_1.87.2-1709912201_amd64.deb 

cd 
source .bashrc
mkdir -p rmvision/src && cd rmvision/src
git clone https://github.com/PnX-HKUSTGZ/auto-aim.git --depth 1
git clone https://github.com/PnX-HKUSTGZ/vision_bringup.git --depth 1
git clone --depth 1  https://github.com/PnX-HKUSTGZ/rm_serial_driver.git
git clone --depth 1  https://github.com/PnX-HKUSTGZ/rm_gimbal_description.git
git clone --depth 1  https://github.com/PnX-HKUSTGZ/hik-driver.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to rm_vision_bringup
source ~/rmvision/install/setup.bash
