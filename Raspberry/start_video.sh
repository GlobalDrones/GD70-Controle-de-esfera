!bash

sudo ip addr flush dev eth0
sudo ip addr add 192.168.144.100/24 dev eth0;
sudo ip link set eth0 up;
sudo arp-scan --localnet;

cd \~mediamtx/;
./mediamtx;
