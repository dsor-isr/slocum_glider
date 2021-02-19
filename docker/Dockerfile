# This Dockerfile builds a "kitchen-sink" production ready image. This image is
# useful for deployment to desktop computers, but is not ideal for deployment
# to resource constrained robots. The workspace containing the code is located
# at /slocum_glider_ws and the entrypoint is overridden to source the install
# space of this workspace.

ARG ros_distro=melodic
ARG frl_msgs_commit=ab362d2f7264ce8be4d6b12bf41538ca4215bfba

FROM ros:$ros_distro-ros-base as deps-extractor
# Dependency management in ROS has always been a painful experience IMHO. We
# want to install all our dependencies based on the package.xml files so that
# we don't have to duplicate them here and keep two independent lists of deps
# up to date. But we don't want to simply run rosdep after adding the source
# code because if there's any change in the source code that would bust
# Docker's cache and make us install all the deps again. Ugh.
#
# Therefore, we use multi-stage build instead! In the first image, we import
# all our source code, run rosdep in simulation mode to get an install script,
# sort the script (this may not be necessary... not sure how rosdep traverses
# the dependency tree...), copy the script to the next image, and run it
# *before* copying in our source code.

ARG ros_distro
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Initialize the workspace
WORKDIR /slocum_glider_ws/src

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.bash" \
    && catkin_init_workspace

# Get the devel, etc. folders created
WORKDIR /slocum_glider_ws/

# hadolint ignore=SC1091
RUN . "/opt/ros/$ros_distro/setup.bash" \
    && catkin_make

# We have to install FRL msgs manually as it's not available via a package
# manager.
ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit
RUN . "/opt/ros/$ros_distro/setup.bash" \
    && git clone https://github.com/Field-Robotics-Lab/frl_msgs.git \
    && git -C frl_msgs archive --prefix=src/frl_msgs/ "$FRL_MSGS_COMMIT" | tar x \
    && rm -rf frl_msgs

# Copy in all the source code:
COPY slocum_glider src/slocum_ros/slocum_glider
COPY slocum_glider_extctl src/slocum_ros/slocum_glider_extctl
COPY slocum_glider_launch src/slocum_ros/slocum_glider_launch
COPY slocum_glider_msgs src/slocum_ros/slocum_glider_msgs
COPY slocum_glider_extctl src/slocum_ros/slocum_glider_extctl
COPY slocum_glider_extctl_serial src/slocum_ros/slocum_glider_extctl_serial
COPY slocum_glider_extctl_sim src/slocum_ros/slocum_glider_extctl_sim
COPY slocum_glider_mission_controller src/slocum_ros/slocum_glider_mission_controller

# Generate the install script, run it, install any deps it doesn't pick up, and
# build the code.
RUN . "/opt/ros/$ros_distro/setup.bash" \
    && apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends python-pip \
    && rosdep install --from-paths src --ignore-src -y -s --as-root pip:false --as-root apt:false | sort -u | tee /tmp/install-script \
    && sh /tmp/install-script \
    && rm -rf /var/lib/apt/lists/*

# Assemble the builder.
FROM ros:$ros_distro-ros-base as builder

ARG ros_distro
COPY --from=deps-extractor /tmp/install-script /tmp/install-script

# Install the dependencies
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends python-pip \
    && cat /tmp/install-script \
    && sh /tmp/install-script \
    && rm /tmp/install-script \
    # The version of six in apt-get is too old
    && pip install six==1.15.0 \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Carry forward the FRL msgs commit as an env var in this image
ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit

# Copy in the source code. We *could* just do all the COPY commands and git
# clone frl_msgs again, but why bother since we already have all that available
# in the previous stage...
WORKDIR /slocum_glider_ws
COPY --from=deps-extractor /slocum_glider_ws/src src

# make and install!
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && catkin_make \
    && catkin_make install

# Assemble the runtime image.
FROM ros:$ros_distro-ros-base

ARG ros_distro
COPY --from=deps-extractor /tmp/install-script /tmp/install-script

# Install the dependencies
RUN . "/opt/ros/$ros_distro/setup.sh" \
    && apt-get update \
    && apt-get install -y --no-upgrade --no-install-recommends python-pip \
    && cat /tmp/install-script \
    && sh /tmp/install-script \
    && rm /tmp/install-script \
    # The version of six in apt-get is too old
    && pip install six==1.15.0 \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Carry forward the FRL msgs commit as an env var in this image
ARG frl_msgs_commit
ENV FRL_MSGS_COMMIT=$frl_msgs_commit

# Copy in the install space:
COPY --from=builder /slocum_glider_ws/install /slocum_glider_ws/install

# Overwrite the entrypoint so that our overlay is sourced.
COPY docker/prod_entrypoint.sh /ros_entrypoint.sh
