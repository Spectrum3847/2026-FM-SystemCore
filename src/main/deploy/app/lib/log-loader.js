/** Shared log picker: open a .wpilog from this computer. Used by Logs, Power and CAN. */
import { el, fmtBytes } from "./ui.js";
import { parseWpilog } from "./wpilog.js";
import { LogModel, analysisKeep } from "./log-model.js";

/**
 * @param {{onLoad: (log, name: string) => void, keep?: (name: string, type: string) => boolean}} opts
 *   keep defaults to the channels the analysis pages read; pass () => true to keep everything.
 */
export function logPicker({ onLoad, keep = analysisKeep }) {
    const file = el("input", { type: "file", accept: ".wpilog", style: "display:none" });
    const status = el("span", { style: "color:var(--tx-faint);font-size:0.8rem" });

    const box = el("div", { class: "row" },
        el("label", { style: "color:var(--tx-dim);font-size:0.84rem" }, "Log"),
        el("button", { onclick: () => file.click() }, "Open a .wpilog…"),
        file,
        status);

    // Only the most recent pick may render.
    let seq = 0;
    let failure = null;

    async function load(f) {
        const my = ++seq;
        failure?.remove();
        failure = null;
        try {
            status.textContent = "reading…";
            const buf = await f.arrayBuffer();
            if (my !== seq) return;
            status.textContent = "parsing…";
            await new Promise((r) => setTimeout(r, 0)); // let the status paint before we block
            if (my !== seq) return;
            const t0 = performance.now();
            const log = parseWpilog(buf, { keep });
            if (log.error && !log.counts.size) throw new Error(log.error);
            status.textContent = `${f.name} · ${fmtBytes(buf.byteLength)} · ${log.durationSec.toFixed(0)}s · parsed in ${(performance.now() - t0).toFixed(0)} ms`;
            onLoad(log, f.name);
        } catch (e) {
            if (my !== seq) return;
            status.textContent = "";
            failure = el("div", { class: "notice bad", style: "flex-basis:100%" },
                el("strong", {}, `Could not show ${f.name}. `), e.message);
            box.append(failure);
            console.error(e);
        }
    }

    file.addEventListener("change", () => {
        const f = file.files?.[0];
        if (!f) return;
        file.value = "";
        load(f);
    });

    return box;
}

export { LogModel };
