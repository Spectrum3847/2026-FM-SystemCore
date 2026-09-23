# System-wide CPU on the SystemCore with the robot program running.
# Run from the repo:  ./gradlew deployrobotLogsystemcore -ProbotLog -ProbotTop --info
# GradleRIO stops the robot program before a command artifact, so this starts it again and lets it
# settle first. top's second sample is the one to read (the first covers time since boot).
sudo systemctl start robot
sleep ${ROBOT_TOP_SETTLE:-180}
echo "cores: $(nproc)  load: $(cat /proc/loadavg)"
top -b -n 2 -d 5 > /tmp/top.txt 2>&1
# BusyBox top: each frame starts with a Mem: line.
awk '/^Mem:/{n++} n==2' /tmp/top.txt | cut -c1-150 | head -30
