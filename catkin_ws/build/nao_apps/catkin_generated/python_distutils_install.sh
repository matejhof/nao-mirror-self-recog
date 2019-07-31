#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/nao_robot/nao_apps"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/vojta/code-nao-simulation/gazebo9/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/vojta/code-nao-simulation/gazebo9/catkin_ws/install/lib/python2.7/dist-packages:/home/vojta/code-nao-simulation/gazebo9/catkin_ws/build/nao_apps/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/vojta/code-nao-simulation/gazebo9/catkin_ws/build/nao_apps" \
    "/usr/bin/python2" \
    "/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/nao_robot/nao_apps/setup.py" \
    build --build-base "/home/vojta/code-nao-simulation/gazebo9/catkin_ws/build/nao_apps" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/vojta/code-nao-simulation/gazebo9/catkin_ws/install" --install-scripts="/home/vojta/code-nao-simulation/gazebo9/catkin_ws/install/bin"
