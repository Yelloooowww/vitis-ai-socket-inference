#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#


IMG=argnctu/pokingbot-rl:husky

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
docker exec -it \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    ${containerid} \
    bash
xhost -
