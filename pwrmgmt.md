## Power Management for Jetson

Troubleshooting guide for Wifi and Realsense camera related power issues on the Mantis Drone

**Setting**:
The on board computer(OBC) on the drone and your laptop are both connected to the same network. Currently (05/02/24) this network is 'TP-Link_5F80_5G'. The drone has a realsense camera attached connected to it via USB 3.0.

**Problem**: When pinging the OBC IP address (<client_ip>) from the laptop (<server_ip>), latency is pretty high and you can get a ping of upto 300ms. In certain settings, when the realsense camera is already connected while booting up, the ping might even reach 550ms. You might see a pattern like this with these problems:

```bash
PING <client_ip> (<client_ip>) 56(84) bytes of data.
64 bytes from <client_ip>: icmp_seq=1 ttl=64 time=145 ms
64 bytes from <client_ip>: icmp_seq=2 ttl=64 time=167 ms
64 bytes from <client_ip>: icmp_seq=3 ttl=64 time=4.67 ms
64 bytes from <client_ip>: icmp_seq=4 ttl=64 time=2.34 ms
64 bytes from <client_ip>: icmp_seq=5 ttl=64 time=28.0 ms
64 bytes from <client_ip>: icmp_seq=6 ttl=64 time=50.7 ms
64 bytes from <client_ip>: icmp_seq=7 ttl=64 time=71.9 ms
64 bytes from <client_ip>: icmp_seq=8 ttl=64 time=95.2 ms
64 bytes from <client_ip>: icmp_seq=9 ttl=64 time=117 ms
64 bytes from <client_ip>: icmp_seq=10 ttl=64 time=141 ms
64 bytes from <client_ip>: icmp_seq=11 ttl=64 time=166 ms
64 bytes from <client_ip>: icmp_seq=12 ttl=64 time=189 ms
64 bytes from <client_ip>: icmp_seq=13 ttl=64 time=3.19 ms
64 bytes from <client_ip>: icmp_seq=14 ttl=64 time=24.7 ms
64 bytes from <client_ip>: icmp_seq=15 ttl=64 time=47.7 ms
64 bytes from <client_ip>: icmp_seq=16 ttl=64 time=71.0 ms
64 bytes from <client_ip>: icmp_seq=17 ttl=64 time=92.4 ms
64 bytes from <client_ip>: icmp_seq=18 ttl=64 time=117 ms
64 bytes from <client_ip>: icmp_seq=19 ttl=64 time=138 ms
64 bytes from <client_ip>: icmp_seq=20 ttl=64 time=161 ms
64 bytes from <client_ip>: icmp_seq=21 ttl=64 time=189 ms
64 bytes from <client_ip>: icmp_seq=22 ttl=64 time=212 ms
64 bytes from <client_ip>: icmp_seq=23 ttl=64 time=23.9 ms
```

**Reason**: To save power, the wifi interface on the Jetson Xavier NX computer onboard  the mantis drone goes into power saving mode. You can check if this is true by using the `iwconfig` command in the Jetson terminal and seeing if Power Management is 'on'.
Output:
```bash
wlan0     IEEE 802.11  ESSID:"TP-Link_5F80_5G"  
          Mode:Managed  Frequency:5.2 GHz  Access Point: 62:61:B4:4C:5F:7F   
          Bit Rate=-2.02647e+06 kb/s   Tx-Power=22 dBm   
          Retry short limit:7   RTS thr:off   Fragment thr:off
          Power Management:on
          Link Quality=67/70  Signal level=-43 dBm  
          Rx invalid nwid:0  Rx invalid crypt:0  Rx invalid frag:0
          Tx excessive retries:0  Invalid misc:36   Missed beacon:0
```

Note that the common solution to set `wifi.powersave = 2` in `/etc/NetworkManager/default-wifi-powersave.conf` does not work. It would show that power mangement is off, but still the ping would be high! This is a common issue with jetson pcs (see [ff](https://github.com/robwaat/Tutorial/blob/master/Jetson%20Disable%20Wifi%20Power%20Management.md)).

To set power management off manually:
```bash
sudo iwconfig wlan0 power off
```

But in case a realsense camera is also connected to the PC, even the manual solution doesn't work sometimes. The hypothesis is that the camera reconfigures the interface for some reason. This also happens with USB ports apparently ([ref](https://forums.developer.nvidia.com/t/intel-realsense-d435i-showing-as-connected-to-usb-2-1/183851)).

**Solution**
When you see patterns like this, its usually the right sequence of events and settings that need to happen on the OBC in order for the solution to work.
What works reprodibly is to:
1. Power on the OBC
2. Wait for the wifi to connect
3. Reset the realsense from software (python script using pyrealsense2). You might need to customise if you have multiple cameras connected
```python
import pyrealsense2 as rs2
ctx = rs2.context()
list = ctx.query_devices()

for dev in list:
	dev.hardware_reset()
```
4. Wait for a bit and set power management to off manually. 
```
sleep 2
sudo iwconfig wlan0 power off
```

A script called `rs2_wifi_power.sh` which does the above things has been placed in the home dir. 

A service (`/etc/systemd/system/rs2_wifi_power.service`) can be created to run this script on boot up. But note that we need to wait for the wifi to start up and the network to be online to do all this. Otherwise it doesnt work :(
```bash
[Unit]
Description=Execute this on boot
Requires=network-online.target
After=netowrk-onine.target

[Service]
ExecStartPre= /bin/sh -c 'until ping -c1 192.168.0.1; do sleep 1; done;'
ExecStart=/home/drone/rs2_wifi_power.sh
User=root

[Install]
WantedBy=multi-user.target
```  
The service pings the router (192.168.0.1) before starting up because the network-online.target requirement sometimes fails ([ref](https://askubuntu.com/questions/1363944/systemd-only-run-service-after-internet-is-up)).

To test and enable the script:
```bash
sudo systemctl start rs2_wifi_power.service
sudo systemctl enable rs2_wifi_power.service
```

If everything went fine, when you power on the jetson, the realsense resets, power management is turned off, and when you ping the jetson you should get an average ping of about 3ms (when nothing is running) on a 5G router in the drone lab.