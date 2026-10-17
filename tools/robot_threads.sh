# Which threads of the robot program use the CPU, and in which library.
# Run from the repo:  ./gradlew deployrobotLogsystemcore -ProbotLog -ProbotThreads --info
# GradleRIO stops the robot program before a command artifact, so this starts it again, lets it
# settle, samples every thread for 5 s, and maps each busy thread's program counter to the shared
# library it is executing in (a native thread that never named itself shows up as "java").
sudo systemctl start robot
sleep 45
# The unit runs /home/systemcore/robotCommand (bash), which starts the JVM.
pid=""
for p in /proc/[0-9]*; do
    [ "$(cat $p/comm 2>/dev/null)" = java ] && pid=${p#/proc/}
done
echo "pid $pid: $(sudo cat /proc/$pid/cmdline | tr '\000' ' ' | cut -c1-100)"
snap() { for t in /proc/$pid/task/*; do echo "$(basename $t) $(sudo cut -d' ' -f14,15 $t/stat)"; done; }
snap > /tmp/t0; sleep 5; snap > /tmp/t1
echo "threads: $(wc -l < /tmp/t0)"
awk 'NR==FNR {a[$1]=$2+$3; next} ($1 in a) {d=$2+$3-a[$1]; if (d>2) print $1, d/5}' /tmp/t0 /tmp/t1 |
    sort -k2 -nr | head -12 |
while read tid pct; do
    name=$(cat /proc/$pid/task/$tid/comm 2>/dev/null)
    echo "== $tid $name ${pct}% wchan=$(sudo cat /proc/$pid/task/$tid/wchan 2>/dev/null)"
    for i in 1 2 3 4 5 6; do
        sc=$(sudo cat /proc/$pid/task/$tid/syscall 2>/dev/null)
        pc=$(echo "$sc" | awk '{print $NF}')
        lib=""
        if [ "$sc" != "running" ] && [ -n "$pc" ]; then
            lib=$(sudo cat /proc/$pid/maps | awk -v pc=$((pc)) '{split($1,r,"-"); if (strtonum("0x" r[1]) <= pc && pc < strtonum("0x" r[2])) print $6}' | head -1)
        fi
        echo "   syscall: $(echo $sc | cut -c1-40) $lib"
        sleep 0.2
    done
    sudo cat /proc/$pid/task/$tid/stack 2>/dev/null | head -6
done
echo "== real-time threads (policy 1=FIFO 2=RR, rt_priority)"
for t in /proc/$pid/task/*; do
    f=$(sudo cat $t/stat 2>/dev/null) || continue
    rest=${f##*) }
    set -- $rest
    prio=${38}; pol=${39}
    [ "$pol" != "0" ] && echo "$(basename $t) $(cat $t/comm) policy=$pol rt=$prio"
done
echo "== image"
cat /etc/os-release 2>/dev/null | grep -iE "^(NAME|VERSION|PRETTY|BUILD|IMAGE)" ; ls /etc/*release* /etc/*version* 2>/dev/null; cat /etc/limelight* /etc/systemcore* 2>/dev/null | head -5
