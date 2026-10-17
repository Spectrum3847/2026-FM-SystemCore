import { mountHeader, el } from "./lib/ui.js";

mountHeader();

const CARDS = [
    { href: "pages/logs/", title: "Logs", body: "Open a .wpilog from this laptop and see what it holds: duration, mechanisms found, and keys no page shows yet." },
    { href: "pages/power/", title: "Power", body: "Per-motor current, battery sag, pack resistance, energy per mechanism, and breaker trip simulation." },
    { href: "pages/can/", title: "CAN Bus", body: "Bus utilization and error counters for each SystemCore CAN port, and motors that stopped answering." },
    { href: "pages/swerve-align/", title: "Swerve Align", body: "Read the CANcoders live over NetworkTables and get the offsets line to paste into the robot config." },
    { href: "pages/cameras/", title: "Cameras", body: "Live view of each Limelight, its mount checked by accelerometer and AprilTag solve, and auto-tuned exposure." },
];

document.getElementById("cards").append(
    ...CARDS.map((c) =>
        el("a", { href: c.href, class: "card", style: "color:inherit" },
            el("div", { style: "font-weight:650;margin-bottom:4px" }, c.title),
            el("div", { style: "color:var(--tx-dim);font-size:0.85rem" }, c.body))
    )
);
