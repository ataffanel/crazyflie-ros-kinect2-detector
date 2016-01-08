# crazyflie-ros-kinect2-detector

Crazyflie kinect2 detector ROS package. Port of the windows/zmq detector from
https://github.com/bitcraze/kinect-detector-windows for ros.

Requires a marker under the Crazyflie2. The marker and hardware setup is
on the [bitcraze wiki](https://wiki.bitcraze.io/doc:crazyflie:vision:setup#hardware_setup).

The detector publishes the detection image on /detector/threshold and the
Crazyflie pose on transform base_link.

The pose only contains x,y,z and yaw. Roll and pitch are 0.
