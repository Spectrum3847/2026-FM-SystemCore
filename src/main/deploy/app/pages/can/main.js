import { mountHeader, el } from "../../lib/ui.js";
import { logPicker, LogModel } from "../../lib/log-loader.js";
import { clipToEnabled, seriesStats } from "../../lib/log-model.js";
import { timeChart, xy, decimate, enabledShadePlugin, limitLinePlugin, seriesColor, theme, withAlpha } from "../../lib/charts.js";
import { connectionDropouts, staleTraces, alertEvents, counterSteps } from "../../lib/analyze-can.js";

mountHeader();
const root = document.getElementById("app");
const charts = [];

const content = el("div", {});
root.replaceChildren(
    el("h1", {}, "CAN bus health"),
    el("p", { class: "lede" },
        "Bus load and error counters for each CAN bus, and motors that stopped answering. ",
        "From each leader's Connected flag and each follower's FollowerConnected array."),
    el("div", { class: "card", style: "margin-bottom:20px" }, logPicker({ onLoad: render })),
    content
);

const tile = (label, value, note, cls) =>
    el("div", { class: "stat" },
        el("div", { class: "label" }, label),
        el("div", { class: `value ${cls || ""}` }, value),
        note ? el("div", { class: "note" }, note) : null);

const fmtWindows = (windows) =>
    windows.slice(0, 6).map(([s, e]) => `${s.toFixed(1)}–${e.toFixed(1)}s`).join(", ") + (windows.length > 6 ? ` +${windows.length - 6} more` : "");

function chartCard(title, note, height = "") {
    const canvas = el("canvas");
    const card = el("section", {},
        el("h2", {}, title),
        note ? el("div", { class: "section-note" }, note) : null,
        el("div", { class: "card" }, el("div", { class: `chart-box ${height}` }, canvas)));
    return { card, canvas };
}

function render(log) {
    for (const c of charts.splice(0)) c.destroy();
    const m = new LogModel(log);
    const shade = enabledShadePlugin(m.enabled, log.firstTs, log.lastTs);
    const blocks = [];

    const buses = m.canBuses().map((name) => {
        const { util, tec, rec, busOff, txFull } = m.busTelemetry(name);
        return { name, util, tec, rec, busOffSteps: counterSteps(busOff), txFullSteps: counterSteps(txFull) };
    });
    const loop = m.loopTimeMs();

    const dropouts = connectionDropouts(m);
    const stale = staleTraces(m);
    const alerts = alertEvents(m);

    // ------------------------------------------------------------ no CAN telemetry at all
    if (!buses.length) {
        blocks.push(el("div", { class: "notice warn" },
            el("strong", {}, "No CAN bus telemetry in this log. "),
            "Robot logs <bus>/BusUtilization once a second on the robot only, so sim logs have none. ",
            "The motor-dropout analysis below still works."));
    }

    // ------------------------------------------------------------ tiles
    const maxOf = (series) => (series.length ? Math.max(...series.map(([, v]) => v)) : null);
    const maxAcross = (pick) => buses.reduce((a, b) => (maxOf(pick(b)) === null ? a : Math.max(a ?? 0, maxOf(pick(b)))), null);
    const busiest = buses
        .map((b) => ({ name: b.name, stats: seriesStats(clipToEnabled(b.util, m.enabled).length ? clipToEnabled(b.util, m.enabled) : b.util) }))
        .filter((b) => b.stats)
        .sort((a, b) => b.stats.max - a.stats.max)[0];
    const uStats = busiest?.stats ?? null;
    const maxTec = maxAcross((b) => b.tec);
    const maxRec = maxAcross((b) => b.rec);
    const busOffCount = buses.reduce((n, b) => n + b.busOffSteps.length, 0);
    const txFullCount = buses.reduce((n, b) => n + b.txFullSteps.length, 0);
    const deadMotors = new Set(dropouts.map((d) => d.motor));

    blocks.push(
        el("div", { class: "grid cols-4", style: "margin-bottom:6px" },
            tile("Peak bus load", uStats ? `${uStats.max.toFixed(0)}%` : "n/a",
                uStats ? `${busiest.name}, ${uStats.mean.toFixed(0)}% average` : "not logged",
                uStats ? (uStats.max > 70 ? "bad" : uStats.max > 50 ? "warn" : "ok") : ""),
            tile("Peak TX errors", maxTec !== null ? maxTec : "n/a", "bus-off at 255",
                maxTec === null ? "" : maxTec > 200 ? "bad" : maxTec > 50 ? "warn" : "ok"),
            tile("Peak RX errors", maxRec !== null ? maxRec : "n/a", "bus-off at 255",
                maxRec === null ? "" : maxRec > 200 ? "bad" : maxRec > 50 ? "warn" : "ok"),
            tile("Bus-off events", buses.length ? busOffCount : "n/a", "controller reset the bus",
                busOffCount ? "bad" : buses.length ? "ok" : ""),
            tile("TX queue full", buses.length ? txFullCount : "n/a", "frames dropped before sending",
                txFullCount ? "warn" : buses.length ? "ok" : ""),
            tile("Motors that went dark", deadMotors.size, deadMotors.size ? [...deadMotors].join(", ") : "none detected",
                deadMotors.size ? "bad" : "ok"))
    );

    const deadFollowers = m.mechanisms.filter((x) => x.deadFollowers.length);
    if (deadFollowers.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Followers that dropped"),
                el("div", { class: "section-note" }, "From each mechanism's FollowerConnected array, while enabled. Its leader carried the load alone."),
                ...deadFollowers.map((x) =>
                    el("div", { class: "notice bad" },
                        el("strong", {}, `${x.displayName}: `),
                        `follower ${x.deadFollowers.map((i) => `#${i + 1}`).join(", ")} read disconnected.`)))
        );
    }

    if (dropouts.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Leader disconnected"),
                el("div", { class: "section-note" }, "The direct signal, from each mechanism's Connected input."),
                ...dropouts.map((d) =>
                    el("div", { class: "notice bad" },
                        el("strong", {}, `${d.motor}: `),
                        `disconnected for ${d.totalSec.toFixed(1)}s across ${d.windows.length} window${d.windows.length === 1 ? "" : "s"}. `,
                        el("div", { style: "margin-top:5px;color:var(--tx-faint);font-size:0.8rem" }, fmtWindows(d.windows)))))
        );
    }

    if (!dropouts.length && !deadFollowers.length) {
        blocks.push(el("div", { class: "notice" },
            el("strong", {}, "No motor dropouts detected. "),
            "No leader or follower read disconnected."));
    }

    // ------------------------------------------------------------ utilization
    if (buses.some((b) => b.util.length)) {
        const { card, canvas } = chartCard("Bus utilization",
            "CTRE wants this well under 60%. High load delays frames and produces the stale-frame warnings that precede an unresponsive motor.");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "%",
            yMax: 100,
            datasets: buses.filter((b) => b.util.length).map((b, i) => ({
                label: b.name, data: xy(decimate(b.util)), borderColor: seriesColor(i), fill: buses.length === 1, backgroundColor: withAlpha(seriesColor(i), 0.12),
            })),
            plugins: [shade, limitLinePlugin([
                { value: 50, color: theme().warn, label: "50% watch", dash: [3, 5] },
                { value: 70, color: theme().bad, label: "70% too high" },
            ])],
        })));
    }

    // ------------------------------------------------------------ error counters
    if (buses.some((b) => b.tec.length || b.rec.length)) {
        const { card, canvas } = chartCard("CAN error counters",
            "A controller goes bus-off at 255 and stops talking entirely. These decay when the bus is healthy, so a rising trend matters more than any single value.",
            "short");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "count",
            datasets: buses.flatMap((b, i) => [
                b.tec.length && { label: `${b.name} transmit`, data: xy(decimate(b.tec)), borderColor: seriesColor(i), fill: false },
                b.rec.length && { label: `${b.name} receive`, data: xy(decimate(b.rec)), borderColor: seriesColor(i), borderDash: [4, 3], fill: false },
            ]).filter(Boolean),
            plugins: [shade, limitLinePlugin([{ value: 255, label: "bus-off" }])],
        })));
    }

    // ------------------------------------------------------------ loop time
    if (loop.length) {
        const periodMs = m.loopPeriodSec() * 1000;
        const overruns = loop.filter(([, v]) => v > periodMs).length;
        const { card, canvas } = chartCard("Loop time",
            `AdvantageKit's full cycle time. ${((100 * overruns) / loop.length).toFixed(1)}% of loops ran longer than the ${periodMs.toFixed(0)} ms period. ` +
            "A CAN stall shows up here as a spike, because a blocking status-signal read holds up the whole loop.",
            "short");
        blocks.push(card);
        queueMicrotask(() => charts.push(timeChart(canvas, {
            yTitle: "ms",
            datasets: [{ label: "full cycle", data: xy(decimate(loop)), borderColor: seriesColor(0), fill: false }],
            plugins: [shade, limitLinePlugin([{ value: periodMs, color: theme().warn, label: `${periodMs.toFixed(0)} ms period` }])],
        })));
    }

    // ------------------------------------------------------------ stale traces
    if (stale.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Suspicious silences"),
                el("div", { class: "section-note" },
                    "Advisory only. Logging is change-based, so a controller that stops answering goes quiet rather than flat. " +
                    "But so does a mechanism that is simply idle. Read these alongside what the robot was doing at the time."),
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {}, ["Motor", "Normal interval", "Gaps", "Longest gap", "When"].map((h) => el("th", {}, h)))),
                        el("tbody", {}, stale.map((s) =>
                            el("tr", {},
                                el("td", {}, s.motor),
                                el("td", { class: "num" }, `${s.medianIntervalMs.toFixed(0)} ms`),
                                el("td", { class: "num" }, s.count),
                                el("td", { class: "num" }, `${s.gaps[0][2].toFixed(1)}s`),
                                el("td", { class: "mono", style: "font-size:0.75rem;color:var(--tx-faint)" },
                                    `${s.gaps[0][0].toFixed(1)}–${s.gaps[0][1].toFixed(1)}s`)))))))
        );
    }

    // ------------------------------------------------------------ alerts
    if (alerts.length) {
        blocks.push(
            el("section", {},
                el("h2", {}, "Alerts"),
                el("div", { class: "section-note" }, "Each line is when that alert became active."),
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {}, ["Time", "Level", "Message"].map((h) => el("th", {}, h)))),
                        el("tbody", {}, alerts.map((a) =>
                            el("tr", {},
                                el("td", { class: "num" }, `${a.t.toFixed(1)}s`),
                                el("td", {}, el("span", { class: `tag ${a.level === "error" ? "bad" : a.level === "warning" ? "warn" : ""}` }, a.level)),
                                el("td", {}, a.text)))))))
        );
    }

    content.replaceChildren(...blocks);
}
