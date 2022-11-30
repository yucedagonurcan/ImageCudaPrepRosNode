# CUDA Image Processing ROS Node
This is a ROS implementation of a fairly common image processing pipeline with using CUDA and NPP:
1. Debayer
2. Undistort(Rectify)
3. Resize

## Used Packages & Resources
1. NPP library for **Debayer** & **Resize** classes
2. Code structure and function of **Undistort** class are highly related to the [dusty-nv/jetson-utils](https://github.com/dusty-nv/jetson-utils)

## Run in your machine

1. Go to nearest ROS workspace and clone this repo: 
```shell
cd catkin_ws/src
git clone https://github.com/yucedagonurcan/ImageCudaPrepRosNode.git
```
2. Build the node:
```shell
catkin build cuda_img_processing --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Example run
![Example Run](materials/example_run.gif)





