import { mountHeader, el, fmtDuration } from "../../lib/ui.js";
import { logPicker, LogModel } from "../../lib/log-loader.js";
import { clipToEnabled, seriesStats, fitInternalResistance } from "../../lib/log-model.js";
import { timeChart, xy, decimate, enabledShadePlugin, limitLinePlugin, seriesColor, theme, withAlpha, Chart } from "../../lib/charts.js";
import { simulateBreaker } from "../../lib/breaker.js";

// FRC main breaker rating.
const BREAKER_AMPS = 120;

mountHeader();
const root = document.getElementById("app");
const charts = [];

const content = el("div", {});
root.replaceChildren(
    el("h1", {}, "Power"),
    el("p", { class: "lede" },
        "Where the battery goes: pack current and sag, the main breaker, and current and energy per mechanism. ",
        "Current limits are compiled into the Java and never appear in the log, so this page does not show them."),
    el("div", { class: "card", style: "margin-bottom:20px" }, logPicker({ onLoad: render })),
    content
);

function tile(label, value, note, cls) {
    return el("div", { class: "stat" },
        el("div", { class: "label" }, label),
        el("div", { class: `value ${cls || ""}` }, value),
        note ? el("div", { class: "note" }, note) : null);
}

function chartCard(title, note, height = "") {
    const canvas = el("canvas");
    const card = el("section", {},
        el("h2", {}, title),
        note ? el("div", { class: "section-note" }, note) : null,
        el("div", { class: "card" }, el("div", { class: `chart-box ${height}` }, canvas)));
    return { card, canvas };
}

function render(log, name) {
    for (const c of charts.splice(0)) c.destroy();
    const m = new LogModel(log);
    const windows = m.enabled;
    const tMin = log.firstTs;
    const tMax = log.lastTs;
    const shade = enabledShadePlugin(windows, tMin, tMax);

    const volts = m.batteryVoltage();
    const amps = m.totalCurrent();
    const voltsEnabled = clipToEnabled(volts, windows);
    const ampsEnabled = clipToEnabled(amps, windows);
    const vStats = seriesStats(voltsEnabled.length ? voltsEnabled : volts);
    const aStats = seriesStats(ampsEnabled.length ? ampsEnabled : amps);
    const energy = m.totalEnergyWh();
    const resistance = fitInternalResistance(volts, amps);
    const breaker = amps.length ? simulateBreaker(amps, { ratedAmps: BREAKER_AMPS }) : null;
    const brownout = m.brownoutVolts() ?? 6.0;

    const blocks = [];

    // ------------------------------------------------------------ warnings
    for (const w of m.warnings) blocks.push(el("div", { class: "notice warn" }, w));

    // ------------------------------------------------------------ tiles
    blocks.push(
        el("div", { class: "grid cols-4", style: "margin-bottom:6px" },
            tile("Enabled", fmtDuration(m.enabledSec), `of ${fmtDuration(log.durationSec)} logged`),
            tile("Peak current", aStats ? `${aStats.max.toFixed(0)} A` : "n/a", aStats ? `${aStats.mean.toFixed(0)} A average` : null,
                aStats && aStats.max > 300 ? "warn" : ""),
            tile("Min voltage", vStats ? `${vStats.min.toFixed(2)} V` : "n/a", `brownout set to ${brownout} V`,
                vStats ? (vStats.min < brownout + 1.5 ? "bad" : vStats.min < 8 ? "warn" : "ok") : ""),
            tile("Energy used", energy !== null ? `${energy.toFixed(1)} Wh` : "n/a", null),
            tile("Pack resistance", resistance ? `${(resistance.ohms * 1000).toFixed(1)} mΩ` : "n/a",
                resistance ? `open-circuit ${resistance.openCircuitVolts.toFixed(2)} V` : "needs voltage + current",
                resistance ? (resistance.ohms > 0.025 ? "bad" : resistance.ohms > 0.018 ? "warn" : "ok") : ""),
            tile("Main breaker", breaker ? `${breaker.peakPct.toFixed(0)}%` : "n/a",
                breaker ? (breaker.trips.length ? `${breaker.trips.length} simulated trip${breaker.trips.length === 1 ? "" : "s"}` : "of trip threshold") : null,
                breaker ? (breaker.trips.length ? "bad" : breaker.peakPct > 60 ? "warn" : "ok") : ""))
    );

    // ------------------------------------------------------------ pack charts
    //
    // Current and voltage are shown as two charts sharing one x-range rather than one chart with
    // two y-scales. Two scales on one plot let whoever picks the scales imply any correlation they
    // want, and here the honest story, current spikes and voltage sags, reads perfectly well
    // stacked.
    if (amps.length || volts.length) {
        const xRange = [log.firstTs, log.lastTs];
        if (amps.length) {
            const { card, canvas } = chartCard("Pack current",
                "Shaded stretches are enabled. Everything outside them is the robot sitting on a cart.");
            blocks.push(card);
            queueMicrotask(() => charts.push(timeChart(canvas, {
                yTitle: "amps",
                xRange,
                datasets: [{ label: "total current", data: xy(decimate(amps)), borderColor: seriesColor(0), fill: false }],
                plugins: [shade],
            })));
        }
        if (volts.length > 1) {
            const { card, canvas } = chartCard("Battery voltage",
                `Same time range. The dashed line is this robot's configured brownout threshold, ${brownout} V.`,
                "short");
            blocks.push(card);
            queueMicrotask(() => charts.push(timeChart(canvas, {
                yTitle: "volts",
                xRange,
                yBeginAtZero: false,
                datasets: [{ label: "battery", data: xy(decimate(volts)), borderColor: seriesColor(3), fill: false }],
                plugins: [shade, limitLinePlugin([{ value: brownout, label: `brownout ${brownout} V` }])],
            })));
        }
    }

    // ------------------------------------------------------------ breaker
    if (breaker && breaker.thermal.length) {
        const { card, canvas } = chartCard("Main breaker thermal model",
            `Bussmann ${BREAKER_AMPS} A I²t accumulator against the datasheet trip curve. ` +
            "100% means it would have popped. The curve is the slow end of the tolerance band, so this errs toward saying you survived.",
            "short");
        blocks.push(card);
        if (breaker.trips.length) {
            card.append(el("div", { class: "notice bad" },
                el("strong", {}, `Simulated trip at ${breaker.trips.map((t) => `${t.toFixed(1)}s`).join(", ")}. `),
                "Sustained current above the breaker rating for long enough to heat it out."));
        }
        queueMicrotask(() => {
            charts.push(timeChart(canvas, {
                yTitle: "% of trip",
                yMax: 105,
                datasets: [{ label: "breaker heat", data: xy(decimate(breaker.thermal)), borderColor: theme().bad, fill: true, backgroundColor: withAlpha(theme().bad, 0.14) }],
                plugins: [shade, limitLinePlugin([{ value: 100, label: "trip" }])],
            }));
        });
    }

    // ------------------------------------------------------------ per-motor table
    if (m.mechanisms.length && m.hasPerMotorCurrent) {
        const rows = m.mechanisms.map((mech) => {
            const sStats = seriesStats(clipToEnabled(mech.stator, windows));
            const pStats = seriesStats(clipToEnabled(mech.supply, windows));
            const tStats = seriesStats(mech.temp);
            const energyS = mech.batteryEnergy;
            const wh = energyS.length ? energyS[energyS.length - 1][1] : null;

            return el("tr", { class: sStats ? null : "dim" },
                el("td", {}, mech.displayName,
                    mech.followerCount ? el("span", { class: "tag", style: "margin-left:6px", title: "Supply current includes the followers" }, `+${mech.followerCount}`) : null,
                    mech.deadFollowers.length ? el("span", { class: "tag bad", style: "margin-left:6px" }, "follower dropped") : null),
                el("td", { class: "num" }, sStats ? `${sStats.max.toFixed(0)} A` : "n/a"),
                el("td", { class: "num" }, sStats ? `${sStats.mean.toFixed(1)} A` : "n/a"),
                el("td", { class: "num" }, pStats ? `${pStats.max.toFixed(0)} A` : "n/a"),
                el("td", { class: "num" }, tStats ? `${tStats.max.toFixed(0)} °C` : "n/a"),
                el("td", { class: "num" }, wh !== null ? `${wh.toFixed(1)} Wh` : "n/a"));
        });

        blocks.push(
            el("section", {},
                el("h2", {}, "Per-motor current"),
                el("div", { class: "section-note" },
                    "Current statistics cover enabled time only. Stator is the leader's; supply adds every follower's (the +N tag)."),
                el("div", { class: "card table-wrap" },
                    el("table", {},
                        el("thead", {}, el("tr", {},
                            ["Motor", "Peak stator", "Mean stator", "Peak supply", "Peak temp", "Energy"]
                                .map((h) => el("th", {}, h)))),
                        el("tbody", {}, rows))))
        );

        // -------------------------------------------------------- per-motor chart
        const plotted = new Set(m.mechanisms.filter((x) => x.stator.length || x.supply.length).slice(0, 4).map((x) => x.name));
        const { card, canvas } = chartCard("Motor current over time", "Click a motor to add or remove it.", "tall");
        const picker = el("div", { class: "motor-pick" });
        card.querySelector(".card").prepend(picker);
        blocks.push(card);

        const drawMotors = () => {
            picker.replaceChildren(...m.mechanisms.map((mech) =>
                el("button", {
                    class: plotted.has(mech.name) ? "on" : "",
                    onclick: () => { plotted.has(mech.name) ? plotted.delete(mech.name) : plotted.add(mech.name); drawMotors(); },
                }, mech.displayName)));

            for (let i = charts.length - 1; i >= 0; i--) {
                if (charts[i].canvas === canvas) charts.splice(i, 1)[0].destroy();
            }
            const chosen = m.mechanisms.filter((x) => plotted.has(x.name));
            charts.push(timeChart(canvas, {
                yTitle: "amps (stator)",
                datasets: chosen.map((mech, i) => ({
                    label: mech.displayName,
                    data: xy(decimate(mech.stator.length ? mech.stator : mech.supply)),
                    borderColor: seriesColor(i),
                    fill: false,
                })),
                plugins: [shade],
            }));
        };
        queueMicrotask(drawMotors);

        // -------------------------------------------------------- energy split
        const withEnergy = m.mechanisms.filter((x) => x.batteryEnergy.length).map((x) => ({
            name: x.displayName,
            wh: x.batteryEnergy[x.batteryEnergy.length - 1][1],
        })).filter((x) => x.wh > 0.001).sort((a, b) => b.wh - a.wh);

        if (withEnergy.length) {
            const canvas2 = el("canvas");
            blocks.push(
                el("section", {},
                    el("h2", {}, "Energy by mechanism"),
                    el("div", { class: "section-note" },
                        "Cumulative watt-hours over the whole log, follower motors included. This is the honest \"what drains the battery\" ranking."),
                    el("div", { class: "card" }, el("div", { class: "chart-box" }, canvas2)))
            );
            queueMicrotask(() => {
                charts.push(new Chart(canvas2, {
                    type: "bar",
                    data: {
                        labels: withEnergy.map((x) => x.name),
                        datasets: [{
                            label: "Wh",
                            data: withEnergy.map((x) => +x.wh.toFixed(2)),
                            backgroundColor: withEnergy.map((_, i) => seriesColor(i)),
                            borderRadius: 4,
                            borderSkipped: "start",
                        }],
                    },
                    options: {
                        indexAxis: "y",
                        maintainAspectRatio: false,
                        plugins: { legend: { display: false } },
                        scales: { x: { title: { display: true, text: "watt-hours" }, grid: { color: theme().grid } }, y: { grid: { display: false } } },
                    },
                }));
            });
        }
    }

    content.replaceChildren(...blocks);
}
