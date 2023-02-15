# Azure Kinect ROS Driver

This project is a node which publishes sensor data from the [Azure Kinect Developer Kit](https://azure.microsoft.com/en-us/services/kinect-dk/) to the [Robot Operating System (ROS)](http://www.ros.org/). Developers working with ROS can use this node to connect an Azure Kinect Developer Kit to an existing ROS installation.

This repository uses the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) to communicate with the Azure Kinect DK. It supports both Linux and Windows installations of ROS.

[![Build Status](https://dev.azure.com/ms/Azure_Kinect_ROS_Driver/_apis/build/status/microsoft.Azure_Kinect_ROS_Driver?branchName=melodic)](https://dev.azure.com/ms/Azure_Kinect_ROS_Driver/_build/latest?definitionId=166&branchName=melodic)

## Features

This ROS node outputs a variety of sensor data, including:

- A PointCloud2, optionally colored using the color camera
- Raw color, depth and infrared Images, including CameraInfo messages containing calibration information
- Rectified depth Images in the color camera resolution
- Rectified color Images in the depth camera resolution
- The IMU sensor stream
- A TF2 model representing the extrinsic calibration of the camera

The camera is fully configurable using a variety of options which can be specified in ROS launch files or on the command line.

However, this node does ***not*** expose all the sensor data from the Azure Kinect Developer Kit hardware. It does not provide access to:

- Microphone array

For more information about how to use the node, please see the [usage guide](docs/usage.md).

## Status

This code is provided as a starting point for using the Azure Kinect Developer Kit with ROS. Community developed features are welcome.

For information on how to contribute, please see our [contributing guide](CONTRIBUTING.md).

## Building

The Azure Kinect ROS Driver uses catkin to build. For instructions on how to build the project please see the 
[building guide](docs/building.md).

