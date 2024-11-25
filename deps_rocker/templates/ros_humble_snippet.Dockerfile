#Add PPA or apt sources, or source code

# # Env setup
# ENV ROS_SYNC_DATESTAMP="2024-11-04"

ENV LANG=en_US.UTF-8
ENV ROS_PYTHON_VERSION=3
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
# disable terminal interaction for apt
ENV DEBIAN_FRONTEND=noninteractive
# ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp


RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"


RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# ROS fundamentals
RUN --mount=type=cache,target=/var/cache/apt \
apt-get update && apt-get install -y \
        python3-colcon-common-extensions \
        python3-pip \
        python3-rosdep \
        python3-vcstool 



# # Avoid setup.py and easy_install deprecation warnings caused by colcon and setuptools
# # https://github.com/colcon/colcon-core/issues/454
# ENV PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::pkg_resources,ignore:::setuptools.command.develop
# RUN echo "Warning: Using the PYTHONWARNINGS environment variable to silence setup.py and easy_install deprecation warnings caused by colcon"

# ENV ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/${ROS_DISTRO}/${ROS_SYNC_DATESTAMP}/index-v4.yaml

# # Add ROS 2 apt repository
# RUN --mount=type=cache,target=/var/cache/apt \
#     apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key 4B63CF8FDE49746E98FA01DDAD19BAB3CBF125EA && \
#     echo "deb http://snapshots.ros.org/${ROS_DISTRO}/${ROS_SYNC_DATESTAMP}/ubuntu $(lsb_release -sc) main" >> /etc/apt/sources.list.d/ros-snapshots.list && \
#     apt-get update