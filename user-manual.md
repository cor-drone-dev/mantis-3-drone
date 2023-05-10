# User Manual

## Onboard Computer

Password: cor-drones

## WiFi Router

Name: COR-AMR
Password: cor-drones

## ROS Multi-Machine Communication

1. Use `hostname` to query the hostname of your computer. E.g. you get `laptop` for your laptop and `drone1-desktop` for your drone.
2. Type `ifconfig` to query the IP address. E.g. you get `192.168.1.100` for your laptop and `192.168.1.101` for your drone.
3. Modify the `/etc/hosts` file on both machines (you will need sudo access), fill in the IP address and corresponding hostname.
4. Edit the `~/.bashrc` file on both machines, configure ROS_HOSTNAME and ROS_MASTER_URI variables as follows:
   ```
   export ROS_HOSTNAME=<hostname-of-this-machine>
   export ROS_MASTER_URI=http://<hostname-of-master-machine>:11311
   ```
   Note: The master machine is the machine where you start `roscore`. If you run `roscore` in the wrong machine, you will receive a warning when you start the `roscore`.
5. Remember to `source` the updated `~/.bashrc` file.
   Note: If there are some mistakes in the settings, you will receive an error message like this: _ROS_HOSTNAME / ROS_IP is set to only allow local connections, so a requested to `<some-machine>` is being rejected_. The ROS message from the remote machine may not be accepted to the master machine.
