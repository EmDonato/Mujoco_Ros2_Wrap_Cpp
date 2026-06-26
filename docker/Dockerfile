FROM ros:jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV FASTRTPS_DEFAULT_PROFILES_FILE=
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git\
    libglfw3-dev \
    wget \
    ros-jazzy-rmw-cyclonedds-cpp\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN wget -q https://github.com/google-deepmind/mujoco/releases/download/3.9.0/mujoco-3.9.0-linux-x86_64.tar.gz && \
    tar -xzf mujoco-3.9.0-linux-x86_64.tar.gz && \
    rm mujoco-3.9.0-linux-x86_64.tar.gz

ENV MUJOCO_PATH=/opt/mujoco-3.9.0
ENV LD_LIBRARY_PATH=/opt/mujoco-3.9.0/lib
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /ws

CMD ["/bin/bash"]