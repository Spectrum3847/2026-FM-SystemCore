import { mountHeader, el, fmtDuration } from "../../lib/ui.js";
import { logPicker } from "../../lib/log-loader.js";
import { analysisKeep, enabledWindows, MECH, BUS, META, ENABLED, SWERVE_ALIGN } from "../../lib/log-model.js";

mountHeader();
const root = document.getElementById("app");
const content = el("div", {});

const METADATA = ["RobotIdentity", "Robot", "GitSHA", "GitBranch", "GitDate", "BuildDate", "RuntimeMode", "LogFolder"];

// Values are only kept for what this page reads; every other key still gets its name, type and count.
const keep = (name) => name.startsWith(META) || name === ENABLED;

root.replaceChildren(
    el("h1", {}, "Logs"),
    el("p", { class: "lede" },
        "Open a ", el("code", {}, ".wpilog"), " to see what it holds. The robot writes them to ",
        el("code", {}, "/U/logs"), " on a USB stick, else ", el("code", {}, "/home/systemcore/logs"), ". ",
        "Copy one off with ", el("code", {}, "scp systemcore@robot.local:/U/logs/<file> ."), " or pull the stick."),
    el("div", { class: "card", style: "margin-bottom:20px" }, logPicker({ onLoad: render, keep })),
    content
);

const tile = (label, value, note) =>
    el("div", { class: "stat" },
        el("div", { class: "label" }, label),
        el("div", { class: "value" }, value),
        note ? el("div", { class: "note" }, note) : null);

function render(log) {
    const meta = (k) => {
        const s = log.channel(`${META}${k}`);
        return s.length ? String(s[s.length - 1][1]) : null;
    };
    const names = [...log.counts.keys()].filter((n) => !n.startsWith(META));
    const enabledSec = enabledWindows(log).reduce((a, [s, e]) => a + (e - s), 0);
    const mechanisms = [...new Set(names.map((n) => MECH.exec(n)?.[1]).filter(Boolean))].sort();
    const buses = names.map((n) => BUS.exec(n)?.[1]).filter(Boolean).sort();
    // Keys no page reads, grouped by their first two path parts, so new telemetry is easy to spot.
    const unread = new Map();
    for (const n of names) {
        if (analysisKeep(n) || n.startsWith(SWERVE_ALIGN)) continue;
        const group = n.split("/").slice(0, 3).join("/");
        unread.set(group, (unread.get(group) || 0) + 1);
    }
    const records = names.reduce((a, n) => a + (log.counts.get(n) || 0), 0);

    content.replaceChildren(
        log.error ? el("div", { class: "notice warn" }, el("strong", {}, "Log ends early. "), log.error) : null,
        el("div", { class: "grid cols-4", style: "margin-bottom:6px" },
            tile("Duration", fmtDuration(log.durationSec), `${fmtDuration(enabledSec)} enabled`),
            tile("Keys", names.length.toLocaleString(), `${records.toLocaleString()} records`),
            tile("Mechanisms", mechanisms.length, mechanisms.join(", ") || "none"),
            tile("CAN buses", buses.length, buses.join(", ") || "none logged")),
        el("section", {},
            el("h2", {}, "Run"),
            el("div", { class: "card table-wrap" },
                el("table", {},
                    el("tbody", {}, METADATA.filter((k) => meta(k) !== null).map((k) =>
                        el("tr", {}, el("td", {}, k), el("td", { class: "mono" }, meta(k)))))))),
        el("section", {},
            el("h2", {}, "Keys no page shows"),
            el("div", { class: "section-note" }, "Grouped by path. Open the log in AdvantageScope to see these."),
            el("div", { class: "card table-wrap" },
                el("table", {},
                    el("thead", {}, el("tr", {}, ["Group", "Keys"].map((h) => el("th", {}, h)))),
                    el("tbody", {}, [...unread].sort((a, b) => a[0].localeCompare(b[0])).map(([g, n]) =>
                        el("tr", {}, el("td", { class: "mono" }, g), el("td", { class: "num" }, n))))))));
}
