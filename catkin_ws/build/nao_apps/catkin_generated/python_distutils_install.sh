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

echo_and_run cd "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/nao_robot/nao_apps"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/lib/python2.7/dist-packages:/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_apps/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_apps" \
    "/usr/bin/python" \
    "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/nao_robot/nao_apps/setup.py" \
    build --build-base "/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/build/nao_apps" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install" --install-scripts="/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/install/bin"
