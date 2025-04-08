# PX4 ROS2 Implementation
This guide is designed to assist you in starting and utilizing a ROS 2 Gazebo Harmonic simulation for practicing autonomy, control, and visual perception tasks. While the PX4 Gazebo simulation may not be optimal for algorithms such as optical flow or feature point extraction, it performs adequately for Apriltag and object detection in addition to basic autonomy. 

## Simulation Prerequisites and Downloads:
### PX4 Simulator
https://docs.px4.io/main/en/robotics/ 

Choose some new spot/folder on your computer and make a folder named 'ros2_simulation' or similar, cd into it, and run the following:
```bash
apt update && apt install git

git clone https://github.com/PX4/PX4-Autopilot.git -b main --recursive

./Tools/setup/ubuntu.sh

#then run this:

pip uninstall em && pip install empy==3.3.4
```

### Gazebo Simulation

You have everything you need now to also try the Gazebo simulation, which is higher fidelity:

at the PX4_Autopilot root...
```bash
make px4_sitl gz_x500_depth

```
If a window called "Gazebo Sim" starts up then you have successfully started a Gazebo Garden simulation which is considerably more advanced than jmavsim, but requires more resources to run.  You'll note that the environment is rather boring at this point; simply a blank, grey area.  Use "ctrl +c" to get out of the sim (and exit out of the GUI window if it hangs too long).

Relevant source: https://docs.px4.io/main/en/sim_gazebo_gz/

Next, try a more interesting environment.  This may take a long time to load, depending on your machine:

```bash
make px4_sitl gz_x500_depth_baylands
```

in this command, 'x500' is the vehicle, 'depth' is an additional tag that specifies the x500 with a depth and EO camera mounted, and 'baylands' tells the sim to pull and use the Baylands environment.

Note: from the documentation, you see an equivalent way of starting is: 
```bash
PX4_GZ_WORLD=baylands make px4_sitl gz_x500_depth
```
### QGroundControl

At this point, with the simulation running, you should see a GUI open with a very low fidelity quadrotor ready to take off.  Next, you need to be able to control it.  For that, we use QGroundControl.

https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html

#### Unbuntu Linux
QGroundControl can be installed/run on Ubuntu LTS 20.04 (and later).

Ubuntu comes with a serial modem manager that interferes with any robotics related use of a serial port (or USB serial). Before installing QGroundControl you should remove the modem manager and grant yourself permissions to access the serial port. You also need to install GStreamer in order to support video streaming.

Before installing QGroundControl for the first time:

1. On the command prompt enter:
    ```bash
    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
    sudo apt install libfuse2 -y
    sudo apt install libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor-dev -y
    ```
2. Logout and login again to enable the change to user permissions.
 
To install QGroundControl:
1. Download [QGroundControl.AppImage](https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage).
2. Install (and run) using the terminal commands:
    ```bash
    chmod +x ./QGroundControl.AppImage
    ./QGroundControl.AppImage  (or double click)
    ```
#### Suggestions
I would download the .AppImage and start by either double-clicking the icon, or executing from the command line.  (This would be a good application for having an alias setup in your .bashrc).  After running, the program should connect up automatically to the simulator.  You'll see a little green bar/banner moving from left to right across the top.  If the banner in the upper left is red or yellow, something is wrong or not ready.  Typically, if you see this now, it is because no controller is connected.  For the sim to work, some kind of controller needs to be detected.  For the time being, the easy thing to do is click the 'Q square' menu in the upper left -> go to Application Settings -> tick the 'Virtual Joystick' option.  You'll then note that two circles are back on the main screen.  Left is Throttle and Yaw and right is Roll/Pitch.  Arm by clicking 'Ready for Takeoff' and sliding the arm switch over, then just drag the throttle up for a few seconds.  Alternatively, there is a 'takeoff' command on the left.  We'll get into some of the additional options later.

Trying arming the vehicle (click "ready to fly" and hold the spacebar or drag the circle over) and then takeoff by pushing UP on the THROTTLE.  Play around a bit with the controls to familiarize yourself.
