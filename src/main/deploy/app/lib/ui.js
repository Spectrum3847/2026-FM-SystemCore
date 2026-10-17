/** Shared UI helpers: the top bar, robot status pill, and small DOM conveniences. */

// App root, so links work wherever the app is served from (robot.local:5800/app/ or a file server).
export const ROOT = new URL("../", import.meta.url);

const PAGES = [
    { href: "", label: "Home", match: (p) => p === ROOT.pathname || p === ROOT.pathname + "index.html" },
    { href: "pages/logs/", label: "Logs" },
    { href: "pages/power/", label: "Power" },
    { href: "pages/can/", label: "CAN Bus" },
    { href: "pages/swerve-align/", label: "Swerve Align" },
    { href: "pages/cameras/", label: "Cameras" },
];

/**
 * Light is the brand default (the team site is white with a deep-purple nav). Dark is a second,
 * separately stepped palette for pit and queue-line use, remembered per browser.
 */
export function initTheme() {
    let stored = null;
    try {
        stored = localStorage.getItem("spectrum-theme");
    } catch {
        // Private browsing or blocked storage: fall through to the brand default.
    }
    document.documentElement.dataset.theme = stored === "dark" ? "dark" : "light";
}

export function toggleTheme() {
    const next = document.documentElement.dataset.theme === "dark" ? "light" : "dark";
    document.documentElement.dataset.theme = next;
    try {
        localStorage.setItem("spectrum-theme", next);
    } catch {
        // Not persisting the choice is survivable; failing the click is not.
    }
    document.dispatchEvent(new CustomEvent("themechange", { detail: { theme: next } }));
    const btn = document.querySelector(".theme-toggle");
    if (btn) {
        btn.textContent = next === "dark" ? "\u2600" : "\u263D";
        btn.title = next === "dark" ? "Switch to light theme" : "Switch to dark theme";
    }
}

// Applied before first paint so the page never flashes the wrong theme.
initTheme();

export function el(tag, attrs = {}, ...children) {
    const node = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
        if (v === null || v === undefined || v === false) continue;
        if (k === "class") node.className = v;
        else if (k === "html") node.innerHTML = v;
        else if (k.startsWith("on") && typeof v === "function") node.addEventListener(k.slice(2), v);
        else node.setAttribute(k, v === true ? "" : String(v));
    }
    for (const c of children.flat()) {
        if (c === null || c === undefined || c === false) continue;
        node.append(c instanceof Node ? c : document.createTextNode(String(c)));
    }
    return node;
}

export function mountHeader() {
    const here = location.pathname.replace(/index\.html$/, "");
    const tabs = el(
        "nav",
        { class: "tabs" },
        PAGES.map((p) => {
            const href = new URL(p.href, ROOT).pathname;
            const active = p.match ? p.match(here) : here.startsWith(href);
            return el("a", { href, class: active ? "active" : null }, p.label);
        })
    );
    const dark = document.documentElement.dataset.theme === "dark";
    const themeBtn = el(
        "button",
        {
            class: "theme-toggle",
            title: dark ? "Switch to light theme" : "Switch to dark theme",
            "aria-label": "Toggle colour theme",
            onclick: toggleTheme,
        },
        dark ? "\u2600" : "\u263D"
    );
    const header = el(
        "header",
        { class: "topbar" },
        el("div", { class: "brand" }, el("span", { class: "mark" }, "\u25C5 "), "Spectrum", el("span", { class: "sub" }, "FM on SystemCore")),
        tabs,
        themeBtn
    );
    document.body.prepend(header);
    return header;
}

export function fmtBytes(n) {
    if (n === null || n === undefined) return "n/a";
    if (n < 1024) return `${n} B`;
    if (n < 1024 ** 2) return `${(n / 1024).toFixed(0)} KB`;
    if (n < 1024 ** 3) return `${(n / 1024 ** 2).toFixed(1)} MB`;
    return `${(n / 1024 ** 3).toFixed(2)} GB`;
}

export function fmtDuration(sec) {
    if (sec === null || sec === undefined) return "n/a";
    if (sec < 60) return `${sec.toFixed(0)}s`;
    const m = Math.floor(sec / 60);
    return `${m}m ${Math.round(sec - m * 60)}s`;
}

export function fmtDate(ms) {
    if (!ms) return "n/a";
    return new Date(ms).toLocaleString(undefined, { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" });
}
