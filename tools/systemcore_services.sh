# Run by every deploy (build.gradle, systemCoreServices). Our PhotonVision package (photonvision-sc)
# replaces SystemCore's built-in Limelight vision, which holds the field's team ports 5800-5807 and
# would fight it for the camera. An OS update or reimage turns the built-in vision back on, so this
# turns it off again, but only where photonvision-sc is installed: without it, the built-in cameras
# are all the robot has. Prints one line per finding; always exits 0 so it never stops a deploy.
vision="limelight_visionserver limelight_visionserver1 limelight_visionserver2 limelight_visionserver3"
if ! opkg list-installed 2>/dev/null | grep -q '^photonvision-sc '; then
    echo "SystemCore services: photonvision-sc not installed; left Limelight's built-in vision as is"
    exit 0
fi
turned_off=""
for u in $vision; do
    if systemctl is-enabled -q "$u" 2>/dev/null || systemctl is-active -q "$u" 2>/dev/null; then
        sudo systemctl disable --now "$u" >/dev/null 2>&1
        turned_off="$turned_off $u"
    fi
done
if [ -n "$turned_off" ]; then
    echo "SystemCore services: turned Limelight vision back off (an update re-enabled it):$turned_off"
fi
if systemctl is-enabled -q photonvision-sc && systemctl is-active -q photonvision-sc; then
    echo "SystemCore services: OK (photonvision-sc running, Limelight vision off)"
else
    echo "SystemCore services: WARNING photonvision-sc is installed but not running" \
        "(enabled=$(systemctl is-enabled photonvision-sc 2>&1), active=$(systemctl is-active photonvision-sc))"
fi
exit 0
