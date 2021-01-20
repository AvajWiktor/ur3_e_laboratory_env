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

echo_and_run cd "/home/wiktor/atest_ur/src/fmauch_universal_robot/ur_kinematics"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/wiktor/atest_ur/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/wiktor/atest_ur/install/lib/python2.7/dist-packages:/home/wiktor/atest_ur/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/wiktor/atest_ur/build" \
    "/usr/bin/python2" \
    "/home/wiktor/atest_ur/src/fmauch_universal_robot/ur_kinematics/setup.py" \
     \
    build --build-base "/home/wiktor/atest_ur/build/fmauch_universal_robot/ur_kinematics" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/wiktor/atest_ur/install" --install-scripts="/home/wiktor/atest_ur/install/bin"
