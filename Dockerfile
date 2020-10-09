ARG ros_distro=melodic

FROM ros:$ros_distro-ros-base as builder

ARG ros_distro

# Initialize the workspace
WORKDIR /usr/local/slocum_glider_overlay/src

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_init_workspace

# Get the devel, etc. folders created
WORKDIR /usr/local/slocum_glider_overlay/

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_make

# Copy in all the source code:

COPY slocum_glider /usr/local/slocum_glider_overlay/src/slocum_glider
COPY slocum_glider_serial_driver /usr/local/slocum_glider_overlay/src/slocum_glider_serial_driver
COPY slocum_glider_launch /usr/local/slocum_glider_overlay/src/slocum_glider_launch
COPY slocum_glider_msgs /usr/local/slocum_glider_overlay/src/slocum_glider_msgs

# Install rosdeps, make, and install!
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && catkin_make \
    && catkin_make install -DCMAKE_INSTALL_PREFIX=/opt/ros/slocum_glider_overlay \
    && rm -rf /var/lib/apt/lists/*

FROM ros:$ros_distro-ros-base

COPY --from=builder /opt/ros/slocum_glider_overlay /opt/ros/slocum_glider_overlay

# Install dependencies
# hadolint ignore=SC1091
RUN . "/opt/ros/slocum_glider_overlay/setup.sh" \
    && apt-get update \
    && rosdep install --ignore-src -y slocum_glider slocum_glider_serial_driver slocum_glider_launch slocum_glider_msgs \
    && rm -rf /var/lib/apt/lists/*

# Overwrite the entrypoint so that our overlay is sourced.
COPY docker_entrypoint.sh /ros_entrypoint.sh
