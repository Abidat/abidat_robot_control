FROM osrf/ros:humble-desktop

# Setup environment
ENV ROS_DISTRO humble
ENV USER developer

RUN \
    # Add non root user for development
    useradd --create-home --shell /bin/bash $USER \
    && echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && /sbin/usermod -aG sudo $USER \
    # Add line for sourcing ROS environment.
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc

USER $USER