
# ENVIRONMENT
ubuntu 18.04 version
cmake 3.10 version
CLION 2020.2 version


## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
 use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **my version is 3.4.5**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## g2o csparse
[g2o](https://github.com/RainerKuemmerle/g2o.git) the latest version


## glog  
[glog](https://github.com/google/glog) the latest version
```
sudo apt-get install autoconf automake libtool
```
## gtest 
[gtest](https://github.com/google/googletest.git) the latest version. 
```
sudo apt-get install libgtest-dev
sudo apt-get install cmake
cd /usr/src/gtest
sudo cmake CMakeLists.txt
sudo make
sudo cp *.a /usr/lib
```
## gflags
[gflags](https://github.com/gflags/gflags.git) the latest version
```
git clone https://github.com/gflags/gflags.git
cd gflags
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_SHARED_LIBS=ON -DGFLAGS_NAMESPACE=gflags ../ 
make 
sudo make install
```

# 2.Dataset  
[KITTI dataset](https://pan.baidu.com/s/1S6j2BFwzA2qLXX4n3TPOWg)  extract num: orty
