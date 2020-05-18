FROM docker.lzzhao.app:4567/lzzhao/robond-docker/master

# Set up the workspace
RUN /bin/bash -c "echo 'export HOME=/home/ubuntu' >> /root/.bashrc && source /root/.bashrc" && \
    mkdir -p ~/ros_ws/src && \
    cd $HOME && git clone https://gitlab.lzzhao.app/lzzhao/ros-web-navigation-simulation && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && \
                  cd ~/ros-web-navigation-simulation/catkin_ws/ && \
                  catkin_make && \
                  echo 'source ~/ros-web-navigation-simulation/catkin_ws/devel/setup.bash' >> ~/.bashrc"

# Install Depedencies
RUN rosdep install --from-paths ~/ros-web-navigation-simulation/catkin_ws --ignore-src -r -y
