#Raspi-config
sudo raspi-config nonint do_vnc 0
sudo raspi-config nonint do_onewire 0
sudo raspi-config nonint do_camera 0

#Update and Upgrade
yes | sudo apt-get update
yes | sudo apt-get upgrade


yes | sudo apt-get install build-essential cmake unzip pkg-config
yes | sudo apt-get install libjpeg-dev libpng-dev libtiff-dev

#Install Dependencies
yes | sudo apt-get install build-essential python3 python3-dev python3-pip  python3-virtualenv python-numpy python3-numpy python3-picamera python3-pandas python3-rpi.gpio i2c-tools avahi-utils joystick libopenjp2-7-dev libtiff5-dev gfortran libatlas-base-dev libopenblas-dev libhdf5-dev libhdf5-serial-dev libhdf5-103 git ntp

#Optional - Install OpenCV Dependencies
yes | sudo apt-get install libilmbase-dev libopenexr-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libjasper-dev libwebp-dev libatlas-base-dev gfortran libeigen3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev v4l-utils libxvidcore-dev libx264-dev libxine2-dev libqtgui4 libqtwebkit4 libqt4-test

	
yes | sudo apt-get install libgtk-3-dev
yes | sudo apt-get install mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev


wget  https://bootstrap.pypa.io/get-pip.py
yes | sudo python3 get-pip.py
yes | sudo pip3 install opencv-contrib-python

#Setup Virtual Env
python3 -m virtualenv -p python3 env --system-site-packages
echo "source env/bin/activate" >> ~/.bashrc
. ~/.bashrc
. env/bin/activate

#Install Donkeycar Python Code
cd ~/
sudo rm -rf projects
mkdir ~/projects
cd ~/projects
git clone https://github.com/robocarstore/donkeycar
cd donkeycar
git checkout robocarstore_v43
pip install -e .[pi]
pip install https://github.com/lhelontra/tensorflow-on-arm/releases/download/v2.2.0/tensorflow-2.2.0-cp37-none-linux_armv7l.whl
pip install numpy --upgrade

#Create Donkeycar from Template
donkey createcar --path ~/mycar

