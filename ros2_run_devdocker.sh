if [ $# -eq 0 ]
  then
    echo "No arguments supplied! bash assumed"
    cmd=bash
  else
    cmd=$1
fi

docker run -it --rm --privileged --name oe3036 \
    -p 8000:8000 \
    -p 9090:9090 \
    --env="DISPLAY" \
    --env="ROS_DOMAIN_ID=42" \
    --workdir="/workspaces/mavlab" \
    --volume="$(pwd)":"/workspaces/mavlab" \
    --volume="/dev/shm":"/dev/shm" \
    --volume="/dev/sbg":"/dev/sbg" \
    --volume="/dev/ardusimple":"/dev/ardusimple" \
    --volume="/dev/propeller":"/dev/propeller" \
    --volume="/dev/rudder":"/dev/rudder" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    abhilashiit/oe3036:1.0 $cmd