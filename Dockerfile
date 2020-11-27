# This Dockerfile builds a "kitchen-sink" image. This image is useful for
# development and deployment to desktop computers, but is not ideal for
# deployment to resource constrained robots. The workspace containing the code
# is located at /slocum_glider_workspace and the entrypoint is overridden to
# source the install space of this workspace.

ARG ros_distro=melodic

FROM ros:$ros_distro-ros-base as builder

ARG ros_distro

# Initialize the workspace
WORKDIR /slocum_glider_workspace/src

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_init_workspace

# Get the devel, etc. folders created
WORKDIR /slocum_glider_workspace/

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_make

# Copy in all the source code:
COPY slocum_glider src/slocum_glider
COPY slocum_glider_driver src/slocum_glider_driver
COPY slocum_glider_launch src/slocum_glider_launch
COPY slocum_glider_msgs src/slocum_glider_msgs
COPY slocum_glider_serial_driver src/slocum_glider_serial_driver
COPY slocum_glider_sim_driver src/slocum_glider_sim_driver

# Install rosdeps, frl_msgs, make, and install!
ENV FRL_MSGS_COMMIT=01505e9514957a870f93124fc8279b88925dc325
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends python-pip \
    && git clone https://github.com/Field-Robotics-Lab/frl_msgs.git \
    && git -C frl_msgs archive --prefix=src/frl_msgs/ "$FRL_MSGS_COMMIT" | tar x \
    && rm -rf frl_msgs \
    && rosdep install --from-paths src --ignore-src -r -y \
    # The version of six in apt-get is too old
    && pip install six==1.15.0 \
    && catkin_make \
    && catkin_make install \
    && apt-get remove -y python-pip \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Overwrite the entrypoint so that our overlay is sourced.
COPY docker_entrypoint.sh /ros_entrypoint.sh
