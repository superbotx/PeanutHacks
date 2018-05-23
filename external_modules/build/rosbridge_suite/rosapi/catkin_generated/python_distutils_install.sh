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

echo_and_run cd "/home/ashis/botx_ws/PeanutHacks/external_modules/src/rosbridge_suite/rosapi"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ashis/botx_ws/PeanutHacks/external_modules/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ashis/botx_ws/PeanutHacks/external_modules/install/lib/python3/dist-packages:/home/ashis/botx_ws/PeanutHacks/external_modules/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ashis/botx_ws/PeanutHacks/external_modules/build" \
    "/home/ashis/botx_ws/venv3/bin/python" \
    "/home/ashis/botx_ws/PeanutHacks/external_modules/src/rosbridge_suite/rosapi/setup.py" \
    build --build-base "/home/ashis/botx_ws/PeanutHacks/external_modules/build/rosbridge_suite/rosapi" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ashis/botx_ws/PeanutHacks/external_modules/install" --install-scripts="/home/ashis/botx_ws/PeanutHacks/external_modules/install/bin"
