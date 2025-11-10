#!/bin/bash

help()
{
    echo ""
    echo "Usage: $0 -w dev_ws -i ros2-humble-fedora43 -b"
    echo -e "\t-w Workspace folder name (relative to \$HOME on host and /root in container)"
    echo -e "\t-i Image to build/run"
    echo -e "\t-b Build mode"
    echo -e "\t-r Run mode"
    echo -e "\t-h Show help"
    echo ""
    echo "Fedora 43 optimized - uses Podman and Wayland by default"
    exit 1
}

while getopts "w:i:hbr" opt
do
    case "$opt" in
      w) workspace="$OPTARG" ;;
      i) image="$OPTARG" ;;
      b) should_build=true ;;
      r) should_run=true ;;
      h | ?) help ;;
    esac
done

if [[ -z "$workspace" ]]; then
    echo "Workspace folder is mandatory"
    help
fi

if [[ -z "$image" ]]; then
    echo "Image is mandatory"
    help
fi

if [[ -z "$should_build" ]]; then
    should_build=false
fi

if [[ -z "$should_run" ]]; then
    should_run=false
fi

if [[ "$should_build" == false && "$should_run" == false ]] || [[ "$should_build" == true && "$should_run" == true ]]; then
    echo "You must specify either a build(-b) or run(-r) mode"
    help
elif [[ "$should_run" == true ]]; then
    mkdir -p $HOME/$workspace

    # Detect Wayland or X11 session
    if [[ -n "$WAYLAND_DISPLAY" ]]; then
        echo "Running with Wayland support on Fedora 43"
        DISPLAY_SOCKET="${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}"
        DISPLAY_ENV="-e WAYLAND_DISPLAY=$WAYLAND_DISPLAY -e XDG_RUNTIME_DIR=/tmp -e QT_QPA_PLATFORM=wayland -e GDK_BACKEND=wayland"
        DISPLAY_VOLUME="-v ${DISPLAY_SOCKET}:/tmp/${WAYLAND_DISPLAY}:z"
    else
        echo "Running with X11 support (fallback)"
        xhost +local: 2>/dev/null || true
        DISPLAY_ENV="-e DISPLAY=$DISPLAY"
        DISPLAY_VOLUME="-v /tmp/.X11-unix:/tmp/.X11-unix:z"
    fi

    # Timezone mount - Fedora only has /etc/localtime, not /etc/timezone
    # Note: /etc/localtime is mounted without :z label since it's a system file
    if [[ -f /etc/timezone ]]; then
        TIMEZONE_VOLUME="-v /etc/timezone:/etc/timezone:ro"
    else
        TIMEZONE_VOLUME=""
    fi

    podman run -it \
               --rm \
               --net=host \
               --privileged \
               --security-opt label=disable \
               ${DISPLAY_ENV} \
               -e PYTHONBUFFERED=1 \
               ${TIMEZONE_VOLUME} \
               -v /etc/localtime:/etc/localtime:ro \
               -v $HOME/$workspace:/root/$workspace:z \
               ${DISPLAY_VOLUME} \
               -v $PWD/.session.yml:/root/.session.yml:z \
               -v $PWD/.tmux.conf:/root/.tmux.conf:z \
               --device /dev/dri \
               --device /dev/bus/usb:/dev/bus/usb \
               $image
elif [[ "$should_build" == true ]]; then
   echo "Building image with Podman for Fedora 43..."
   podman build --build-arg WORKSPACE=$workspace -t $image .
fi
