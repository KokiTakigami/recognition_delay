# recognition_delay

**参考程度にしてください。**  

## Prerequisite

[carla-ros-bridge/docker/](https://github.com/carla-simulator/ros-bridge/tree/master/docker)配下のfileを以下のように書き換えてください。  

Dockerfile  
```Dockerfile
ARG CARLA_VERSION
ARG ROS_DISTRO

FROM carlasim/carla:$CARLA_VERSION as carla

FROM ros:$ROS_DISTRO-ros-base

ARG CARLA_VERSION
ARG ROS_DISTRO

ENV CARLA_VERSION=$CARLA_VERSION
ENV DEBIAN_FRONTEND=noninteractive
# add for ros topic connection
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN mkdir -p /opt/carla-ros-bridge/src
WORKDIR /opt/carla-ros-bridge

COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI

COPY requirements.txt /opt/carla-ros-bridge
COPY install_dependencies.sh /opt/carla-ros-bridge
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    bash /opt/carla-ros-bridge/install_dependencies.sh; \
    if [ "$CARLA_VERSION" = "0.9.10" ] || [ "$CARLA_VERSION" = "0.9.10.1" ]; then wget https://carla-releases.s3.eu-west-3.amazonaws.com/Backup/carla-0.9.10-py2.7-linux-x86_64.egg -P /opt/carla/PythonAPI/carla/dist; fi; \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/$(ls /opt/carla/PythonAPI/carla/dist | grep py$ROS_PYTHON_VERSION.)" >> /opt/carla/setup.bash; \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/carla/PythonAPI/carla" >> /opt/carla/setup.bash'

COPY . /opt/carla-ros-bridge/src/

## ↓add recognition delay package 
WORKDIR /opt/carla-ros-bridge/src
RUN git clone https://github.com/KokiTakigami/recognition_delay.git

# add autoware_auto_msgs package
RUN git clone https://github.com/tier4/autoware_auto_msgs.git
WORKDIR /opt/carla-ros-bridge/src/autoware_auto_msgs
RUN git checkout f6642370c6f4f42a5dc0b6c1fc4a21396d4dc34c
WORKDIR /opt/carla-ros-bridge

RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    if [ "$ROS_VERSION" == "2" ]; then colcon build; else catkin_make install; fi'

# replace entrypoint
COPY ./docker/content/ros_entrypoint.sh /
```


build.sh  
```shell
#!/bin/sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_DISTRO="foxy"
CARLA_VERSION=$(cat ${SCRIPT_DIR}/../carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION)

while getopts r:c: flag
do
    case "${flag}" in
        r) ROS_DISTRO=${OPTARG};;
        c) CARLA_VERSION=${OPTARG};;
        *) error "Unexpected option ${flag}" ;;
    esac
done

DOCKER_BUILDKIT=1 docker build \
    -t carla-ros-bridge:$ROS_DISTRO \
    -f Dockerfile ${SCRIPT_DIR}/.. \
    --ssh default \
    --build-arg ROS_DISTRO=$ROS_DISTRO \
    --build-arg CARLA_VERSION=$CARLA_VERSION

```


run.sh
```shell
#!/bin/sh

usage() { echo "Usage: $0 [-t <tag>] [-i <image>]" 1>&2; exit 1; }

# Defaults
DOCKER_IMAGE_NAME="carla-ros-bridge"
TAG="foxy"

# mount for data storage
DATA_ROOT="/home/$USER/Documents/"

while getopts ":ht:i:" opt; do
  case $opt in
    h)
      usage
      exit
      ;;
    t)
      TAG=$OPTARG
      ;;
    i)
      DOCKER_IMAGE_NAME=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND-1))

echo "Using $DOCKER_IMAGE_NAME:$TAG"

docker run \
    -it --rm \
    --net=host \
    --privileged \
    -v $DATA_ROOT:/data/ \
    "$DOCKER_IMAGE_NAME:$TAG" "$@"
```

## How to use.  

### local  

```shell
cd ${CARLA_ROS_BRIDGE_ROOT}/docker
./build.sh
./run.sh
```

### in docker container

```shell
source install/setup.bash
```

then, run carla-ros-bridge
```shell
 ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
 ```

other terminal 
```shell
ros2 run recognition_delay recognition_delay
```






