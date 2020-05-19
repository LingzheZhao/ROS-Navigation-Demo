FROM docker.lzzhao.app:4567/lzzhao/robond-docker/master

# Add NVIDIA GPU support
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,video,utility

# Set up the workspace
RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc" && \
    mkdir -p ~/ros_ws/src && \
    cd $HOME && git clone --recursive https://gitlab.lzzhao.app/lzzhao/ros-web-navigation-simulation && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd ~/ros-web-navigation-simulation/catkin_ws/ && \
                  rosdep install --from-paths . --ignore-src -r -y && \
                  catkin_make && \
                  echo 'source ~/ros-web-navigation-simulation/catkin_ws/devel/setup.bash' >> ~/.bashrc"
