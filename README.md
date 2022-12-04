# docker-ros-cuda-py3
cuda11.5 + ros noetic

## build docker image

```
$ export ROS_PROJECT_NAME=hoge
$ BUILD-DOCKER-IMAGE.sh
```
your docker image name will be `hoge_ros`


## run docker container

```
$ export ROS_PROJECT_NAME=hoge
$ RUN-DOCKER-CONTAINER.sh
```
your docker container name will be `hoge_ros_1`
