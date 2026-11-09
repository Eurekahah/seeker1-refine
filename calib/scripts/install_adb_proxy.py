#!/usr/bin/env python3
"""Install a container-side adb that talks to the Windows host adb server.

Run inside the seeker1 container. Downloads Android platform-tools (Linux)
if needed, then installs /usr/local/bin/adb as a wrapper around adb.real.
"""
import io
import os
import shutil
import subprocess
import urllib.request
import zipfile

URL = "https://dl.google.com/android/repository/platform-tools-latest-linux.zip"
REAL = "/usr/local/bin/adb.real"
WRAP = "/usr/local/bin/adb"


def version(path):
    try:
        out = subprocess.check_output([path, "version"], stderr=subprocess.STDOUT, text=True)
        return out.splitlines()[0]
    except Exception:
        return ""


def main():
    if "1.0.41" not in version(REAL):
        print("downloading platform-tools...")
        data = urllib.request.urlopen(URL, timeout=180).read()
        z = zipfile.ZipFile(io.BytesIO(data))
        z.extractall("/tmp/platform-tools-install")
        shutil.copy("/tmp/platform-tools-install/platform-tools/adb", REAL)
    os.chmod(REAL, 0o755)

    wrapper = (
        "#!/bin/sh\n"
        'export ADB_SERVER_SOCKET="${ADB_SERVER_SOCKET:-tcp:host.docker.internal:5037}"\n'
        'exec /usr/local/bin/adb.real "$@"\n'
    )
    with open(WRAP, "w", newline="\n") as f:
        f.write(wrapper)
    os.chmod(WRAP, 0o755)
    print("installed:", version(REAL))
    print(subprocess.check_output([WRAP, "devices"], text=True))


if __name__ == "__main__":
    main()
