#!/bin/sh

# Install OpenCV 4.5 on Jetson Nano
# Source: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html

# Part 3

# download the latest version
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.1.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.1.zip

# unpack
unzip opencv.zip
unzip opencv_contrib.zip

# some administration to make live easier later on
mv opencv-4.5.1 opencv
mv opencv_contrib-4.5.1 opencv_contrib
# clean up the zip files
rm opencv.zip
rm opencv_contrib.zip

cd ~/opencv
mkdir build
cd build


cmake 	-D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
	-D WITH_OPENCL=OFF \
	-D WITH_CUDA=ON \
	-D CUDA_ARCH_BIN=5.3 \
	-D CUDA_ARCH_PTX="" \
	-D WITH_CUDNN=ON \
	-D WITH_CUBLAS=ON \
	-D ENABLE_FAST_MATH=ON \
	-D CUDA_FAST_MATH=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D ENABLE_NEON=ON \
	-D WITH_QT=OFF \
	-D WITH_OPENMP=ON \
	-D WITH_OPENGL=ON \
	-D BUILD_TIFF=ON \
	-D WITH_FFMPEG=ON \
	-D WITH_GSTREAMER=ON \
	-D WITH_TBB=ON \
	-D BUILD_TBB=ON \
	-D BUILD_TESTS=OFF \
	-D WITH_EIGEN=ON \
	-D WITH_V4L=ON \
	-D WITH_LIBV4L=ON \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D BUILD_NEW_PYTHON_SUPPORT=ON \
	-D BUILD_opencv_python3=TRUE \
	-D OPENCV_GENERATE_PKGCONFIG=ON \
	-D BUILD_EXAMPLES=OFF ..

make -j4

sudo rm -r /usr/include/opencv4/opencv2
sudo make install
sudo ldconfig

# save an additional 275 MB
sudo rm -rf ~/opencv
sudo rm -rf ~/opencv_contrib

# cleaning (frees 300 MB)
make clean
sudo apt-get update

# remove the dphys-swapfile now if you have a small sd card
# sudo /etc/init.d/dphys-swapfile stop
# sudo apt-get remove --purge dphys-swapfile

