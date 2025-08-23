FROM osrf/ros:foxy-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git pkg-config \
    libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev gfortran openexr \
    libatlas-base-dev python3-dev python3-numpy \
    libtbb2 libtbb-dev libdc1394-22-dev ros-foxy-camera-info-manager \
    ros-foxy-ament-package \
    && rm -rf /var/lib/apt/lists/*

# Build and install OpenCV 4.6.0
WORKDIR /tmp
RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
    cd opencv && git checkout 4.6.0 && \
    cd ../opencv_contrib && git checkout 4.6.0 && \
    cd ../opencv && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules .. && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/opencv /tmp/opencv_contrib

RUN echo 'export DISPLAY=host.docker.internal:0.0' >> /root/.bashrc

