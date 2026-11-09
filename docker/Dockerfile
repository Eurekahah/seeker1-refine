FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# The ROS 1 apt key shipped in the old base image has expired.
COPY docker/ros1-latest-archive-keyring.gpg /usr/share/keyrings/ros1-latest-archive-keyring.gpg
COPY deb/ros-noetic-image-undistort_0.0.0-0focal_amd64.deb /tmp/

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        python3-pip \
        python3-usb \
        python3-ruamel.yaml \
        usbutils \
        ros-noetic-web-video-server && \
    dpkg -i /tmp/ros-noetic-image-undistort_0.0.0-0focal_amd64.deb || true && \
    apt-get -f install -y && \
    rm -rf /var/lib/apt/lists/* /tmp/ros-noetic-image-undistort_0.0.0-0focal_amd64.deb

RUN mkdir -p /catkin_ws/src && \
    git clone --depth 1 https://github.com/skohlbr/disparity_image_proc.git /catkin_ws/src/disparity_image_proc

COPY . /catkin_ws/src/seeker1

RUN source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -j"$(nproc)"

COPY docker/entrypoint.sh /usr/local/bin/seeker-entrypoint
COPY docker/seeker-camera.sh /usr/local/bin/seeker-camera
COPY docker/seeker-calibrate.sh /usr/local/bin/seeker-calibrate
COPY docker/seeker-web-view.sh /usr/local/bin/seeker-web-view

RUN chmod +x /usr/local/bin/seeker-entrypoint \
             /usr/local/bin/seeker-camera \
             /usr/local/bin/seeker-calibrate \
             /usr/local/bin/seeker-web-view && \
    printf '%s\n' \
      'source /opt/ros/noetic/setup.bash' \
      'source /catkin_ws/devel/setup.bash' \
      'export LD_LIBRARY_PATH=/catkin_ws/src/seeker1/libs/x86:${LD_LIBRARY_PATH}' \
      >> /root/.bashrc

WORKDIR /catkin_ws
ENTRYPOINT ["/usr/local/bin/seeker-entrypoint"]
CMD ["bash"]
