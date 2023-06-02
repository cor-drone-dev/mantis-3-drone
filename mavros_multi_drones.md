# Controlling Multiple Drones via MAVROS

In this file you will learn how to setup MAVROS for multiple drones on the same ROS network.

For MAVROS setting, you can refer to [Controlling Drone via MAVROS](./mavros.md)

## Connecting to QGroundControl

First, we need to allocate a unique ID to each flight controller.
Connect to drone 1 via cable connection and go to `Vehicle Setup -> Parameters` set the parameter `MAV_SYS_ID` to 1.
Then, switch to drone 2 and set the same parameter to 2.

Go to `Application Settings -> General` and disable **AutoConnect**

Next, go to `Application Settings -> Comm Links` add connection manually.
You can click `Add` button on the bottom and set the name of the connection. Then click `Type`, set UDP connection, change the UDP address to the `gcs_url` address above.
On this page, you have the option to connect or disconnect individual drones.

Finally, save the settings and add the connection settings for the 2nd, 3rd drone.

After setting up all the connections, launch the mavros node on each drone and click `connect` at QGroundControl to connect them one by one.
