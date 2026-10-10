#!/usr/bin/env bash
# Installs the Basler pylon SDK from an archive dropped in this directory,
# following the procedure in the INSTALL file shipped with the release.
#
#   ./install_pylon.sh            install to /opt/pylon (needs sudo)
#   ./install_pylon.sh --local    install to ./pylon, no sudo, sets PYLON_ROOT
#
# Handles all three packagings Basler ships for aarch64:
#   pylon-<ver>_linux-aarch64_setup.tar.gz  outer wrapper around the SDK tarball
#   pylon-<ver>_linux-aarch64.tar.gz        the SDK tarball itself
#   pylon_<ver>_linux-aarch64_debs.tar.gz   Debian packages
set -euo pipefail

cd "$(dirname "$0")"

prefix=/opt/pylon
use_sudo=true
if [ "${1:-}" = "--local" ]; then
    prefix="$PWD/pylon"
    use_sudo=false
elif [ -n "${1:-}" ]; then
    echo "usage: $0 [--local]" >&2
    exit 2
fi

if [ "$(uname -m)" != "aarch64" ]; then
    echo "ERROR: this machine is $(uname -m), but the Jetson build needs an aarch64 archive." >&2
    exit 1
fi

shopt -s nullglob

# A *_setup.tar.gz is only a wrapper; unpack it to get at the real SDK tarball.
for outer in pylon-*_linux-aarch64_setup.tar.gz; do
    echo "==> Unwrapping $outer"
    tar -xzf "$outer"
done

debs=(pylon_*linux-aarch64_debs.tar.gz)
sdk=(pylon-*_linux-aarch64.tar.gz)

if [ ${#sdk[@]} -gt 0 ]; then
    archive="${sdk[0]}"
    echo "==> Installing $archive into $prefix"

    if [ "$use_sudo" = true ]; then
        sudo mkdir -p "$prefix"
        sudo tar -C "$prefix" -xzf "$archive"
        sudo chmod 755 "$prefix"
    else
        mkdir -p "$prefix"
        tar -C "$prefix" -xzf "$archive"
        chmod 755 "$prefix"
    fi

elif [ ${#debs[@]} -gt 0 ]; then
    archive="${debs[0]}"
    echo "==> Debian packaging: $archive"
    rm -rf debs && mkdir debs
    tar -C debs -xzf "$archive"

    mapfile -t packages < <(find debs -name '*.deb')
    if [ ${#packages[@]} -eq 0 ]; then
        echo "ERROR: no .deb files inside $archive." >&2
        exit 1
    fi
    printf '    %s\n' "${packages[@]}"

    # apt rather than dpkg -i so codemeter's dependencies resolve.
    sudo apt-get update
    sudo apt-get install -y "${packages[@]/#/./}"
    prefix=/opt/pylon   # the .deb always installs here

else
    echo "ERROR: no pylon archive found in $PWD" >&2
    echo "Download the Linux ARM 64 bit (aarch64) build from baslerweb.com and put it here." >&2
    echo "See README.md." >&2
    exit 1
fi

# GigE cameras need no udev rules; this is USB3 Vision only. Skipped silently
# when running --local without sudo rights.
if [ "$use_sudo" = true ] && [ -x "$prefix/share/pylon/setup-usb.sh" ]; then
    sudo "$prefix/share/pylon/setup-usb.sh" || true
fi

echo
echo "==> Verifying"
if ! "$prefix/bin/pylon-config" --version; then
    echo "ERROR: $prefix/bin/pylon-config did not run." >&2
    exit 1
fi
"$prefix/bin/pylon-config" --cflags-only-I
"$prefix/bin/pylon-config" --libs-only-l

# The ROS component's FindPylon.cmake defaults to /opt/pylon, so PYLON_ROOT is
# only required for a non-default prefix.
if [ "$prefix" != /opt/pylon ]; then
    echo
    echo "pylon is at $prefix, not /opt/pylon, so the build needs PYLON_ROOT set."
    echo "Run this once, then build from a new shell:"
    echo "    echo 'export PYLON_ROOT=$prefix' >> ~/.bashrc"
fi

echo
echo "Done. Next:"
echo "    cd /home/user-accounts/lee4649/auto-aiming"
echo "    colcon build --packages-up-to pylon_ros2_camera_wrapper prm_launch --cmake-args -DCMAKE_BUILD_TYPE=Release"
