# ROS Multi-Machine Communication

> Test on June 7th: The name you set in `/etc/hosts` doesnot have to be the hostname.

1. Use `hostname` to query the hostname of your computer. E.g. you get `laptop` for your laptop and `drone1-desktop` for your drone.
2. Type `ifconfig` to query the IP address. E.g. you get `192.168.1.100` for your laptop and `192.168.1.101` for your drone.
3. Modify the `/etc/hosts` file on both machines (you will need sudo access), fill in the IP address and corresponding hostname. E.g.
   ```
   192.168.1.100     pop-os
   192.168.1.101     drone1-desktop
   192.168.1.102     drone2-desktop
   192.168.1.103     drone3-desktop
   ```
4. Edit the `~/.bashrc` file on both machines, configure ROS_HOSTNAME and ROS_MASTER_URI variables as follows:
   ```
   export ROS_HOSTNAME=<hostname-of-this-machine>
   export ROS_MASTER_URI=http://<hostname-of-master-machine>:11311
   ```
   Note: The master machine is the machine where you start `roscore`. If you run `roscore` in the wrong machine, you will receive a warning when you start the `roscore`.
5. Remember to `source` the updated `~/.bashrc` file.
   Note: If there are some mistakes in the settings, you will receive an error message like this:

   _ROS_HOSTNAME / ROS_IP is set to only allow local connections, so a requested to `<some-machine>` is being rejected_.

   The ROS message from the remote machine may not be accepted to the master machine.

## Time Synchronization

In the following manuals, I will describe how to synchronize multi machines with the help of `ntp` and `chrony` tools.
We use our laptop `laptop` as master machine, and drone `drone1` as slave client.
The IP address of `laptop` is `server_ip=192.168.1.100`, and the IP address of our slave machine is `client_ip=192.168.1.101`

1. You need to install `ntp` `ntpdate` and `chrony` on both machines. You can try following commands:
   ```
   sudo apt-get update
   sudo apt-get install ntp ntpdate chrony
   ```

### Server Settings

You need to setup `chrony` server on your master machine, here are the steps.
First you should add these lines to the end of `/etc/chrony/chrony.conf` file of your server machine:

```
# make it serve time even if it is not synced (as it can't reach out)
local stratum 8

# allow the IP of your peer to connect
allow <server_ip>
```

Then you should start the service on server machine:

```
sudo service chrony start
```

### Client Settings

Then modify the same file on the clinet machine:

```
sudo nano /etc/chrony/chrony.conf
```

Add these lines:

```
server <server_ip> minpoll 0 maxpoll 5 maxdelay .05
```

You can change maxdelay value as you wish. 0.05 means the maximum delay tolerance is 50ms, which works in most cases.

Now you can restart the service on your clinet machine by

```
sudo service chrony start
```

You can check the time delay with `ntpdate -q <client_ip>`

if they still have a big offset do:

```
sudo /etc/init.d/chrony start
sudo /etc/init.d/chrony stop
```

check again. The offset should now get better.

hope that helps!

Reference: [ROS forum](https://answers.ros.org/question/298821/tf-timeout-with-multiple-machines/)

## Bandwidth Test

We can use iperf to test the communication bandwidth between two robots.
`iperf` is a network test tool
Iperf is a network performance testing tool.
Iperf can test maximum TCP and UDP bandwidth performance, has various parameters and UDP characteristics, can be adjusted as needed, and can report bandwidth, delay jitter, and packet loss

Before start testing, make sure the package `iperf` is installed on both machines.
You may use `sudo apt-get install iperf` to install it.

1. First we need to select one robot as the server and the other as the client.
   `iperf` client will send message to the server with the maximum bandwidth.
2. On the server machine, use following commands to start the process:
   `-s` denotes the server machine, `-p` specifies the port of communication.
   `-i` sets the time interval of test results printed on the screen.
   `-t` gives the time duration of network test0.

   ```
   iperf -s -p 14555 -i 1 -t 10
   ```

3. On the client machine, type following commands to start the test:
   `-c` specifies the client, which read the IP address of the server.
   `-d` will switch to dual test mode, which holds a bi-directional message transimission test.

   ```
   iperf -c 192.168.1.100 -p 14555 -i 1 -t 10 -d
   ```
