/** CAN-health analyses that go beyond reading a channel out of the log. */

/** Merge [start, end] windows that are closer together than `gap`. */
function coalesce(windows, gap = 0.25) {
    if (!windows.length) return [];
    const out = [windows[0].slice()];
    for (const w of windows.slice(1)) {
        const last = out[out.length - 1];
        if (w[0] - last[1] <= gap) last[1] = Math.max(last[1], w[1]);
        else out.push(w.slice());
    }
    return out;
}

/**
 * Windows where a mechanism's leader read disconnected. Followers are in LogModel's
 * deadFollowers.
 */
export function connectionDropouts(model) {
    const out = [];
    for (const mech of model.mechanisms) {
        if (!mech.motorConnected.length) continue;
        const windows = [];
        let open = null;
        for (const [t, v] of mech.motorConnected) {
            if (v === false && open === null) open = t;
            else if (v === true && open !== null) {
                windows.push([open, t]);
                open = null;
            }
        }
        if (open !== null) windows.push([open, model.log.lastTs]);
        if (windows.length) {
            out.push({
                motor: mech.displayName,
                windows: coalesce(windows),
                totalSec: windows.reduce((a, [s, e]) => a + (e - s), 0),
            });
        }
    }
    return out;
}

/**
 * Gaps in a channel's record stream while the robot was enabled.
 *
 * AdvantageKit only writes a record when a value CHANGES, so a controller that has stopped answering
 * produces silence rather than a flat line. That makes a long gap in an otherwise chatty channel
 * a genuine staleness signal, weaker than the two detectors above, because an idle mechanism is
 * also silent, so this is advisory only.
 */
export function staleTraces(model, { minGapSec = 1.0, multiple = 25 } = {}) {
    const out = [];
    for (const mech of model.mechanisms) {
        const series = mech.stator.length ? mech.stator : mech.supply;
        if (series.length < 50) continue;

        // Pairs are only compared inside one enabled window, so the disabled stretch between
        // auto and teleop is not reported as a dropout.
        const pairs = [];
        let count = 0;
        let prev = null;
        let prevWindow = -1;
        for (const [t] of series) {
            const w = model.enabled.findIndex(([s, e]) => t >= s && t <= e);
            if (w === -1) continue;
            count++;
            if (w === prevWindow) pairs.push([prev, t]);
            prev = t;
            prevWindow = w;
        }
        if (count < 50 || !pairs.length) continue;

        const deltas = pairs.map(([a, b]) => b - a).sort((a, b) => a - b);
        const median = deltas[deltas.length >> 1];
        const threshold = Math.max(minGapSec, median * multiple);

        const gaps = [];
        for (const [a, b] of pairs) {
            if (b - a >= threshold) gaps.push([a, b, b - a]);
        }
        if (gaps.length) {
            out.push({
                motor: mech.displayName,
                medianIntervalMs: median * 1000,
                thresholdSec: threshold,
                gaps: gaps.sort((a, b) => b[2] - a[2]).slice(0, 5),
                count: gaps.length,
            });
        }
    }
    return out.sort((a, b) => b.gaps[0][2] - a.gaps[0][2]);
}

/** Alerts as they became active, from the Alerts/Errors, Warnings and Infos arrays. */
export function alertEvents(model) {
    const out = [];
    for (const [key, level] of [["Errors", "error"], ["Warnings", "warning"], ["Infos", "info"]]) {
        let prev = new Set();
        for (const [t, list] of model.ch(`Alerts/${key}`)) {
            const now = new Set(Array.isArray(list) ? list : []);
            for (const text of now) if (!prev.has(text)) out.push({ t, level, text });
            prev = now;
        }
    }
    return out.sort((a, b) => a.t - b.t);
}

/** Step events on a monotonic counter, e.g. BusOffCount going 0 -> 1. */
export function counterSteps(series) {
    const steps = [];
    for (let i = 1; i < series.length; i++) {
        if (series[i][1] > series[i - 1][1]) steps.push({ t: series[i][0], from: series[i - 1][1], to: series[i][1] });
    }
    return steps;
}
