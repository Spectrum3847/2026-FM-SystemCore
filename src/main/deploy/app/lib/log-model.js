/**
 * Normalizes a parsed FM .wpilog (AdvantageKit) into one shape the analysis pages can rely on.
 *
 * Inputs sit at the log root (/Mechanisms/<name>/..., /DriverStation/..., /Runtime/...), outputs
 * under /RealOutputs/. Mechanisms are discovered from the log, so a log from older code still
 * opens. Current limits are never logged, so no page shows them.
 */

const OUT = "/RealOutputs/";
const MECH = /^\/Mechanisms\/([^/]+)\//;

/** Channels the analysis pages read. Pass to parseWpilog's keep() so big logs stay small. */
export function analysisKeep(name) {
    return (
        name.startsWith("/Mechanisms/") ||
        name === "/DriverStation/Enabled" ||
        name === "/Runtime/BatteryVoltage" ||
        name === "/RealMetadata/LoopPeriodSeconds" ||
        name === "/RealOutputs/LoggedRobot/FullCycleMS" ||
        /^\/RealOutputs\/(Alerts|BatteryLogger|SystemStats|CANConfig|CANivore|SystemCoreCAN\d*)\//.test(name) ||
        /^\/RealOutputs\/[^/]+\/(CommandedDegrees|PositionDegrees|CommandedRPM|RPM)$/.test(name)
    );
}

export class LogModel {
    constructor(log) {
        this.log = log;
        this.enabled = enabledWindows(log);
        this.enabledSec = this.enabled.reduce((a, [s, e]) => a + (e - s), 0);
        this.mechanisms = this.#discoverMechanisms();
        this.warnings = [];
        this.#checkWarnings();
    }

    /** A channel by output key ("BatteryLogger/Current") or absolute name ("/Runtime/..."). */
    ch(name) {
        return this.log.channel(name.startsWith("/") ? name : OUT + name);
    }

    has(name) {
        return this.log.has(name.startsWith("/") ? name : OUT + name);
    }

    /**
     * Pick whichever of several candidate channels actually carries data.
     *
     * Presence is not enough: a channel declared and written once would give a confidently wrong
     * "minimum voltage".
     */
    #richest(...names) {
        let best = [];
        let bestName = null;
        for (const n of names) {
            const s = this.ch(n);
            if (s.length > best.length) {
                best = s;
                bestName = n;
            }
        }
        return { series: best, name: bestName };
    }

    /** Battery pack voltage. The Runtime input is logged every loop; BatteryLogger at 10 Hz. */
    batteryVoltage() {
        const r = this.#richest("/Runtime/BatteryVoltage", "BatteryLogger/BatteryVoltage");
        this.batteryVoltageKey = r.name;
        return r.series;
    }

    /** Total pack current. BatteryLogger sums per-mechanism supply current plus fixed overhead. */
    totalCurrent() {
        const r = this.#richest("BatteryLogger/Current");
        this.totalCurrentKey = r.name;
        return r.series;
    }

    /** The controller's brownout threshold, if the log has it. */
    brownoutVolts() {
        const s = this.ch("SystemStats/BrownoutVoltage");
        return s.length ? s[s.length - 1][1] : null;
    }

    /** Main loop period from the run's metadata, in seconds. */
    loopPeriodSec() {
        const s = this.log.channel("/RealMetadata/LoopPeriodSeconds");
        const v = s.length ? Number(s[s.length - 1][1]) : NaN;
        return Number.isFinite(v) && v > 0 ? v : 0.02;
    }

    /** True when the log has no per-motor current channels at all, only BatteryLogger rollups. */
    get hasPerMotorCurrent() {
        return this.mechanisms.some((m) => m.stator.length || m.supply.length);
    }

    totalEnergyWh() {
        const e = this.ch("BatteryLogger/Energy");
        return e.length ? e[e.length - 1][1] : null;
    }

    /** CAN bus labels in this log, e.g. "SystemCoreCAN1" or "CANivore". */
    canBuses() {
        return this.log
            .matching(/^\/RealOutputs\/([^/]+)\/BusUtilization$/)
            .filter((x) => x.values.length)
            .map((x) => x.capture)
            .sort();
    }

    /** One entry per mechanism the log contains. */
    #discoverMechanisms() {
        const found = new Map();
        const ensure = (name) => {
            if (!found.has(name)) {
                found.set(name, {
                    name,
                    displayName: name,
                    stator: [],
                    leaderSupply: [],
                    supply: [],
                    voltage: [],
                    temp: [],
                    batteryCurrent: [],
                    batteryEnergy: [],
                    motorConnected: [],
                    followerCount: 0,
                    deadFollowers: [],
                });
            }
            return found.get(name);
        };

        const fields = {
            StatorCurrentAmps: "stator",
            SupplyCurrentAmps: "leaderSupply",
            Voltage: "voltage",
            TempCelsius: "temp",
            Connected: "motorConnected",
        };
        for (const { name, capture, values } of this.log.matching(MECH)) {
            const field = fields[name.slice(`/Mechanisms/${capture}/`.length)];
            if (field) ensure(capture)[field] = values;
        }
        for (const { capture, values } of this.log.matching(/^\/RealOutputs\/BatteryLogger\/Current\/Mechanisms\/(.+)$/)) {
            ensure(capture).batteryCurrent = values;
        }
        for (const { capture, values } of this.log.matching(/^\/RealOutputs\/BatteryLogger\/Energy\/Mechanisms\/(.+)$/)) {
            ensure(capture).batteryEnergy = values;
        }

        for (const m of found.values()) {
            const followers = this.log.channel(`/Mechanisms/${m.name}/FollowerSupplyCurrentAmps`);
            m.supply = withFollowers(m.leaderSupply, followers);
            m.followerCount = followers.reduce((n, [, v]) => Math.max(n, v?.length ?? 0), 0);
            m.deadFollowers = deadFollowerIndexes(this.log.channel(`/Mechanisms/${m.name}/FollowerConnected`), this.enabled);
        }

        return [...found.values()]
            .filter((m) => m.stator.length || m.supply.length || m.batteryCurrent.length)
            .sort((a, b) => a.displayName.localeCompare(b.displayName));
    }

    #checkWarnings() {
        if (!this.has("BatteryLogger/Current")) {
            this.warnings.push("No total-current channel in this log, so pack-level power and the breaker simulation are unavailable.");
        }
        // Sparse channels produce confident-looking numbers from almost no data. Say so.
        const v = this.batteryVoltage();
        const perSec = this.log.durationSec > 0 ? v.length / this.log.durationSec : 0;
        if (v.length && perSec < 1) {
            this.warnings.push(
                `Battery voltage was logged only ${v.length} time${v.length === 1 ? "" : "s"} across ` +
                    `${this.log.durationSec.toFixed(0)}s (${this.batteryVoltageKey}). Minimum voltage and pack resistance ` +
                    "are unreliable here, because a sag between samples is invisible."
            );
        }
        if (this.mechanisms.length && !this.hasPerMotorCurrent) {
            this.warnings.push("This log has no per-motor current channels, only BatteryLogger rollups.");
        }
        const dead = this.mechanisms.filter((m) => m.deadFollowers.length);
        if (dead.length) {
            this.warnings.push(
                `Followers that dropped off while enabled: ${dead.map((m) => `${m.displayName} #${m.deadFollowers.map((i) => i + 1).join(", #")}`).join("; ")}. ` +
                    "Their leader carried the load alone."
            );
        }
    }
}

/** Leader supply current plus every follower's, sample by sample (one inputs table per loop). */
export function withFollowers(leader, followers) {
    if (!followers.length) return leader;
    const out = new Array(leader.length);
    let fi = 0;
    for (let i = 0; i < leader.length; i++) {
        const [t, v] = leader[i];
        while (fi < followers.length - 1 && followers[fi + 1][0] <= t) fi++;
        const f = followers[fi];
        const extra = f && f[0] <= t && Array.isArray(f[1]) ? f[1].reduce((a, b) => a + b, 0) : 0;
        out[i] = [t, v + extra];
    }
    return out;
}

/** Follower slots that read disconnected at any point while enabled. */
export function deadFollowerIndexes(connected, windows) {
    const dead = new Set();
    for (const [t, v] of connected) {
        if (!Array.isArray(v) || (windows.length && !inWindows(t, windows))) continue;
        v.forEach((ok, i) => {
            if (!ok) dead.add(i);
        });
    }
    return [...dead].sort((a, b) => a - b);
}

export function enabledWindows(log) {
    const ds = log.channel("/DriverStation/Enabled");
    const out = [];
    let open = null;
    for (const [t, v] of ds) {
        if (v && open === null) open = t;
        else if (!v && open !== null) {
            out.push([open, t]);
            open = null;
        }
    }
    if (open !== null) out.push([open, log.lastTs]);
    return out;
}

export function inWindows(t, windows) {
    for (const [s, e] of windows) if (t >= s && t <= e) return true;
    return false;
}

/** Restrict a series to the enabled windows. Disabled time makes every average meaningless. */
export function clipToEnabled(series, windows) {
    if (!windows.length) return series;
    return series.filter(([t]) => inWindows(t, windows));
}

export function seriesStats(series) {
    if (!series.length) return null;
    let min = Infinity;
    let max = -Infinity;
    let sum = 0;
    let n = 0;
    for (const [, v] of series) {
        if (typeof v !== "number" || !Number.isFinite(v)) continue;
        if (v < min) min = v;
        if (v > max) max = v;
        sum += v;
        n++;
    }
    return n ? { min, max, mean: sum / n, n } : null;
}

/**
 * Fraction of samples at or above a threshold, and the total time spent there.
 * Time is accumulated from sample spacing, so uneven logging rates do not skew it.
 */
export function timeAtOrAbove(series, threshold) {
    let samples = 0;
    let seconds = 0;
    for (let i = 0; i < series.length; i++) {
        const [t, v] = series[i];
        if (Math.abs(v) < threshold) continue;
        samples++;
        const next = series[i + 1]?.[0];
        const dt = next !== undefined ? Math.min(next - t, 0.1) : 0.02;
        seconds += dt;
    }
    return { samples, fraction: series.length ? samples / series.length : 0, seconds };
}

/**
 * Least-squares fit of pack voltage against total current: V = V0 - I*R.
 * The slope is the pack's effective internal resistance, which is the single best number for
 * "is this battery tired?" A healthy FRC pack sits near 0.012-0.020 ohm.
 */
export function fitInternalResistance(voltage, current) {
    if (voltage.length < 10 || current.length < 10) return null;
    const pairs = [];
    let ci = 0;
    for (const [t, v] of voltage) {
        while (ci < current.length - 1 && current[ci + 1][0] <= t) ci++;
        const c = current[ci];
        if (c && Math.abs(c[0] - t) < 0.1) pairs.push([c[1], v]);
    }
    if (pairs.length < 10) return null;

    let sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const [x, y] of pairs) {
        sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    const n = pairs.length;
    const denom = n * sxx - sx * sx;
    if (Math.abs(denom) < 1e-9) return null;
    const slope = (n * sxy - sx * sy) / denom;
    const intercept = (sy - slope * sx) / n;
    return { ohms: -slope, openCircuitVolts: intercept, samples: n };
}
