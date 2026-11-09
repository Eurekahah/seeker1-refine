#!/bin/sh
export ADB_SERVER_SOCKET="${ADB_SERVER_SOCKET:-tcp:host.docker.internal:5037}"
exec /usr/local/bin/adb.real "$@"
