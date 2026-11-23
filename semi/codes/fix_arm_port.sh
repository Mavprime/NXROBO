#!/system/bin/sh
# Point /dev/ttyACM0 at the uArm before starting the arm driver.
#
# swiftpro_write_node opens /dev/ttyACM0 unconditionally (the name is compiled
# into the binary; pro_control_nomoveit.launch has no port parameter). The uArm
# controller is an Arduino Mega 2560 (USB idVendor 2341) and after a USB glitch
# or a reboot it can come back as /dev/ttyACM1, which makes the node die with
# "Unable to open port". This links /dev/ttyACM0 to whichever ttyACM node the
# arm actually got, and does nothing when the arm is already on ttyACM0.
#
# Stale links: a link left by an earlier run can outlive the ttyACM it points
# at. If the arm then re-enumerates as ttyACM0 the kernel cannot create the
# real /dev/ttyACM0 node under the old link's name, so /dev/ttyACM0 stays a
# dangling link and the driver still fails with "Unable to open port" even
# though the arm is physically fine. This script removes such a link and
# recreates the device node, so it is safe to run again and again.
#
# mknod is not always permitted: this OpenHarmony root shell has no CAP_MKNOD,
# so `mknod` fails with "Operation not permitted" (seen 2026-11-23). When that
# happens the script makes the kernel recreate the node by re-enumerating the
# arm's USB device through the sysfs `authorized` toggle, then re-detects the
# port (the arm may come back as ttyACM0 or ttyACM1).
#
# Usage (before roslaunch, with no launch running):
#   sh /data/local/codes/fix_arm_port.sh

set -u

ARM_VID=2341
DEV=/dev/ttyACM0

# Print the ttyACM name of the arm, or return 1 if none is present.
find_arm() {
    arm=""
    for t in /sys/class/tty/ttyACM*; do
        [ -e "$t" ] || continue
        usb=$(readlink -f "$t/device")
        [ -f "$usb/idVendor" ] || usb=$(dirname "$usb")
        [ "$(cat "$usb/idVendor" 2>/dev/null)" = "$ARM_VID" ] || continue
        arm=$(basename "$t")
        return 0
    done
    return 1
}

# The USB device directory that owns a ttyACM's idVendor.
usb_dir_for() {
    usb=$(readlink -f "/sys/class/tty/$1/device")
    while [ "$usb" != "/" ] && [ ! -f "$usb/idVendor" ]; do
        usb=$(dirname "$usb")
    done
    echo "$usb"
}

# Deauthorize and reauthorize the arm so the kernel recreates /dev/$arm.
# $1 is the arm's current ttyACM name. Sets $arm to the redetected name.
reenumerate_arm() {
    usb=$(usb_dir_for "$1")
    if [ ! -f "$usb/authorized" ]; then
        echo "no authorized knob at $usb" >&2
        return 1
    fi
    echo 0 > "$usb/authorized" || return 1
    sleep 2
    echo 1 > "$usb/authorized" || return 1
    i=0
    while [ "$i" -lt 10 ]; do
        sleep 1
        if find_arm && [ -c "/dev/$arm" ]; then
            return 0
        fi
        i=$((i + 1))
    done
    return 1
}

# 1. Drop a symlink at $DEV whose target no longer exists. Do this first:
#    while it sits on the name, the kernel cannot create the real node.
if [ -L "$DEV" ]; then
    target=$(readlink "$DEV")
    case "$target" in
        /*) resolved=$target ;;
        *)  resolved=/dev/$target ;;
    esac
    if [ ! -e "$resolved" ]; then
        rm -f "$DEV"
        echo "removed stale $DEV -> $target"
    fi
fi

# 2. Find the arm among the ttyACM devices by USB vendor id. The tty class
#    device points at the USB interface, so walk up until the directory that
#    owns idVendor, but never past the USB device level.
if ! find_arm; then
    echo "uArm not found on any /dev/ttyACM* - check the USB cable/power" >&2
    exit 1
fi

# 3. Make $DEV point at the arm.
if [ "$arm" = "ttyACM0" ]; then
    if [ -c "$DEV" ] && [ ! -L "$DEV" ]; then
        echo "arm already at $DEV"
        exit 0
    fi
    # sysfs knows ttyACM0 but the /dev node is missing: recreate it from the
    # sysfs major:minor instead of linking a name to itself.
    devno=$(cat /sys/class/tty/ttyACM0/dev)
    major=${devno%:*}
    minor=${devno#*:}
    rm -f "$DEV"
    if mknod "$DEV" c "$major" "$minor" 2>/dev/null && chmod 666 "$DEV"; then
        echo "created $DEV device node ($devno)"
    else
        # No CAP_MKNOD in this shell ("Operation not permitted"). Replugging
        # the arm has the same effect: the kernel/ueventd recreates the node.
        echo "mknod not permitted, re-enumerating the arm over USB"
        if ! reenumerate_arm "$arm"; then
            echo "cannot recreate $DEV - unplug and replug the arm USB" >&2
            exit 1
        fi
        echo "arm re-enumerated as $arm"
        if [ "$arm" != "ttyACM0" ]; then
            rm -f "$DEV"
            ln -sf "$arm" "$DEV"
            echo "$DEV -> $arm"
        fi
    fi
else
    # Never replace a real device node; only replace a link from a run.
    if [ -e "$DEV" ] && [ ! -L "$DEV" ]; then
        echo "$DEV is a real device, refusing to replace it" >&2
        exit 1
    fi
    rm -f "$DEV"
    ln -sf "$arm" "$DEV"
    echo "$DEV -> $arm"
fi

if : < "$DEV" 2>/dev/null; then
    echo "$DEV opens"
else
    echo "warning: $DEV exists but cannot be opened - check permissions" >&2
    exit 1
fi
