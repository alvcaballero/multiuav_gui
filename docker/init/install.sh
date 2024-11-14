# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl -y
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y


sudo docker run hello-world
sudo groupadd docker

sudo usermod -aG docker $USER
newgrp docker


docker pull bluenviron/mediamtx
 
sudo apt install tmux -y 
sudo apt-get install -y tmuxinator

sudo apt install vsftpd ftp ufw -y
sudo apt install git-all -y 
cd ~
mkdir work
mkdir work/GCS_media
mkdir work/px4
cd work/px4
git clone https://github.com/alvcaballero/multiuav_gui.git

mkdir catkin_ws/
mkdir catkin_ws/src
cd catkin_ws/src
git clone --recurse-submodules -j8 https://github.com/alvcaballero/multiUAV_system.git
git clone https://github.com/CircusMonkey/ros_rtsp.git
