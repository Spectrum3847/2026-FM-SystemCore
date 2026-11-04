#!/usr/bin/env bash
# Replays a real match log on a Raspberry Pi 5 running the SystemCore image and prints what the robot
# code cost on that CPU: per-thread CPU per loop, allocation per loop, GC, peak memory.
#
#   tools/pi_bench.sh LOG.wpilog [paced] [nt] [jfr] [runs=N]
#
#   paced   run at the loop period (100 Hz) instead of as fast as possible, to compare CPU percent
#           with a match (System/ProcessPercent); as fast as possible is the better A/B number
#   nt      also publish to NetworkTables, as the robot does
#   jfr     record a Java Flight Recording and copy it to build/bench/bench.jfr
#   runs=N  repeat N times (default 1); the first run is not warmer than the rest, each is a new JVM
#
# The log must come from the AdvantageKit version the code uses. A log from before alpha-7
# (microsecond timestamps, e.g. TXMAN1) converts with tools/akit_log_to_ns.py.
#
# The Pi is the bench, never a robot: PI_HOST (default systemcore@172.26.0.1, the USB address) must
# answer as a Raspberry Pi, or the script stops. It stops the Pi's robot service while it runs and
# starts it again after. Needs SSH keys for the systemcore user.
set -euo pipefail
cd "$(dirname "$0")/.."

log=${1:?usage: tools/pi_bench.sh LOG.wpilog [paced] [nt] [jfr] [runs=N]}
shift
paced=false nt=false jfr=false runs=1
for a in "$@"; do
    case $a in
        paced) paced=true ;;
        nt) nt=true ;;
        jfr) jfr=true ;;
        runs=*) runs=${a#runs=} ;;
        *) echo "unknown option $a" >&2; exit 2 ;;
    esac
done

host=${PI_HOST:-systemcore@172.26.0.1}
ssh_() { ssh -o BatchMode=yes -o ConnectTimeout=5 "$host" "$@"; }

model=$(ssh_ 'tr -d "\0" < /proc/device-tree/model 2>/dev/null')
case $model in
    "Raspberry Pi"*) echo "bench: $model at $host" ;;
    *) echo "bench: $host is not a Raspberry Pi ($model); refusing to run" >&2; exit 1 ;;
esac

if ! ./gradlew benchStage -q > build/bench-stage.log 2>&1; then
    tail -30 build/bench-stage.log >&2; echo "bench: build failed" >&2; exit 1
fi

# The simulation HAL in place of the SystemCore one: AdvantageKit replays Driver Station data
# through DriverStationSim, which on the real HAL waits for a Driver Station packet that never
# comes. WPILib's linuxarm64 build runs on the Pi's CPU next to the SystemCore build of the rest.
# The WPILib version is GradleRIO's.
hal_version=$(grep -oE 'GradleRIO" version "[^"]+' build.gradle | cut -d'"' -f3)
hal_zip=build/bench-cache/hal-cpp-$hal_version-linuxarm64.zip
if [ ! -s "$hal_zip" ]; then
    mkdir -p build/bench-cache
    # WPILib has published 2027 alphas under both repositories.
    hal_url=""
    for repo in release release-2027; do
        url=https://frcmaven.wpi.edu/artifactory/$repo/org/wpilib/hal/hal-cpp/$hal_version/$(basename "$hal_zip")
        if curl -sfIL -o /dev/null "$url"; then hal_url=$url; break; fi
    done
    if [ -z "$hal_url" ]; then
        echo "bench: no linuxarm64 HAL for WPILib $hal_version on frcmaven" >&2; exit 1
    fi
    curl -sfL -o "$hal_zip" "$hal_url"
    want=$(curl -sfIL "$hal_url" | tr -d '\r' | awk -F': ' 'tolower($1)=="x-checksum-sha256" {print $2}')
    if [ -z "$want" ] || [ "$(sha256sum "$hal_zip" | cut -d' ' -f1)" != "$want" ]; then
        rm -f "$hal_zip"; echo "bench: sim HAL download failed its checksum" >&2; exit 1
    fi
fi
unzip -joq "$hal_zip" "linux/arm64/shared/libwpiHal.so" "linux/arm64/shared/libwpiHaljni.so" \
    -d build/bench/lib

# The program, its native libraries and the deploy folder, where the simulation HAL looks for it.
tar -C build/bench -cf - classpath lib deploy | ssh_ 'rm -rf fmbench/src fmbench/deploy fmbench/lib fmbench/classpath fmbench/robot.jar &&
    mkdir -p fmbench/src/main && tar -C fmbench -mxf - && mv fmbench/deploy fmbench/src/main/deploy'

name=$(basename "$log")
size=$(stat -c %s "$log")
if [ "$(ssh_ "stat -c %s fmbench/$name 2>/dev/null || echo 0")" != "$size" ]; then
    echo "bench: copying $name ($((size / 1000000)) MB)"
    cat "$log" | ssh_ "cat > fmbench/$name"
fi

jvm="-XX:+UseZGC -Djava.library.path=/home/systemcore/fmbench/lib --enable-native-access=ALL-UNNAMED"
jvm+=" --add-opens java.base/jdk.internal.vm=ALL-UNNAMED --add-opens java.base/java.lang=ALL-UNNAMED"
jvm+=" -Dfm.bench=true -Dfm.benchPaced=$paced -Dfm.benchNt=$nt"
$jfr && jvm+=" -XX:StartFlightRecording=filename=/home/systemcore/fmbench/bench.jfr,settings=profile"

ssh_ 'sudo systemctl stop robot'
trap "ssh_ 'sudo systemctl start robot' || true" EXIT
for i in $(seq 1 "$runs"); do
    echo "bench: run $i of $runs (paced=$paced nt=$nt)"
    ssh_ "cd fmbench && rm -f *_sim.wpilog hs_err_*.log && AKIT_LOG_PATH=/home/systemcore/fmbench/$name \
        java $jvm -cp \"classpath/*\" frc.robot.Main 2>&1 | grep -E '\[Bench\]|Exception|Error|at frc' | head -60"
done
$jfr && scp -q "$host:fmbench/bench.jfr" build/bench/bench.jfr && echo "bench: build/bench/bench.jfr"
