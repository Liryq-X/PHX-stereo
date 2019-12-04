

echo "sleep 60s, make sure the camera is reconnected"
sleep 60

echo "set ip address"

#go to the folder 
cd /home/nvidia/Downloads/ArenaSDK_v0.1.4_Linux_ARM64/ArenaSDK_Linux_ARM64/precompiledExamples

echo "turn off eth2"
sudo ip link set down eth2

echo "get one camera information"
./IpConfigUtility /list

echo "force the ip address of this camera"
./IpConfigUtility /force -i 0 -a 192.167.1.41 -s 255.255.255.0 -g 0.0.0.0

echo "turn on eth2"
sudo ip link set up eth2

echo "sleep 30s, make sure the camera is reconnected"
sleep 30

echo "turn off eth1"
sudo ip link set down eth1

echo "get one camera information"
./IpConfigUtility /list

echo "force the ip address of this camera"
./IpConfigUtility /force -i 0 -a 192.167.2.41 -s 255.255.255.0 -g 0.0.0.0

echo "turn off eth2"
sudo ip link set down eth2

echo "set ip address on eth1 and eth2"
sudo ifconfig eth1 192.167.1.1 netmask 255.255.255.0
sudo ifconfig eth2 192.167.2.1 netmask 255.255.255.0

echo "sleep 5s"
sleep 5

echo "get two cameras information"
./IpConfigUtility /list

echo "change receive buffer"

sudo ifconfig eth1 mtu 9000
sudo ethtool -G eth1 rx 4096

sudo sysctl -w net.ipv4.conf.default.rp_filter=0
sudo sysctl -w net.ipv4.conf.all.rp_filter=0
sudo sysctl -w net.ipv4.conf.eth1.rp_filter=0

sudo ifconfig eth2 mtu 9000
sudo ethtool -G eth2 rx 4096

sudo sysctl -w net.ipv4.conf.default.rp_filter=0
sudo sysctl -w net.ipv4.conf.all.rp_filter=0
sudo sysctl -w net.ipv4.conf.eth2.rp_filter=0

#1MB
#sudo sh -c "echo 'net.core.rmem_default=1048576' >> /etc/sysctl.conf"
#sudo sh -c "echo 'net.core.rmem_max=1048576' >> /etc/sysctl.conf"
#sudo sysctl -p
#2MB
sudo sh -c "echo 'net.core.rmem_default=2097152' >> /etc/sysctl.conf"
sudo sh -c "echo 'net.core.rmem_max=2097152' >> /etc/sysctl.conf"
sudo sysctl -p

#roslaunch phenobot_camera start_camera.launch

echo "get GPIO writing permission"
sudo chmod a+x /home/nvidia/Downloads/dqn-tx1-for-nintendo-master/gpio/non_root/setup_gpio.sh




