#!/usr/bin/env bash

# allow access to X server
xhost +

# launcher configuration
PWD=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
SRC_DIR=$PWD/../
DOCKER_IMAGE=tensoft

# create an environment file if does not exist to suppress potential errors
if test ! -f "env_vars"; then
    echo "## File automatically generated to prevent errors!" >> "env_vars"
    echo "## Add the SLACK_WEBHOOK_URL environmental variable to receive notifications on your Slack channel." >> "env_vars"
fi

docker run -it --rm -w "/home/tensoft"\
  --volume ${SRC_DIR}:/home/tensoft:rw\
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --env-file=env_vars \
  --device /dev/dri/ \
  ${DOCKER_IMAGE} /bin/bash
