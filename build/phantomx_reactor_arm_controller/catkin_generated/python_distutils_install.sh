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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/georg/catkin_ws/src/phantomx_reactor_arm/phantomx_reactor_arm_controller"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/georg/catkin_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/georg/catkin_ws/install/lib/python2.7/dist-packages:/home/georg/catkin_ws/build/phantomx_reactor_arm_controller/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/georg/catkin_ws/build/phantomx_reactor_arm_controller" \
    "/usr/bin/python2" \
    "/home/georg/catkin_ws/src/phantomx_reactor_arm/phantomx_reactor_arm_controller/setup.py" \
     \
    build --build-base "/home/georg/catkin_ws/build/phantomx_reactor_arm_controller" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/georg/catkin_ws/install" --install-scripts="/home/georg/catkin_ws/install/bin"
