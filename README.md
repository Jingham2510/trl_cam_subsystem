The subsystem that controls the depth cameras and geotechnical feature mapping in the Terra-robotics laboratory.


REQUIREMENTS:
- realsense2 sdk installed, such that the rust_geomapping package can control the realsense cameras
- opencv installed, such that the aruco tag detection and calibration works (Read here for installation tips: https://github.com/twistedfall/opencv-rust/blob/master/README.md)