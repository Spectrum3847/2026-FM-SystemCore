/**
 * The app has no build step, so nothing else catches a broken relative import or link.
 */
import { test } from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const APP = path.resolve(path.dirname(fileURLToPath(import.meta.url)), "../../src/main/deploy/app");

function files(dir) {
    return fs.readdirSync(dir, { withFileTypes: true }).flatMap((e) => {
        const p = path.join(dir, e.name);
        if (e.isDirectory()) return e.name === "vendor" ? [] : files(p);
        return /\.(html|js|css)$/.test(e.name) ? [p] : [];
    });
}

const PATTERNS = [
    /\bimport\s+(?:[^"']*?\s+from\s+)?["']([^"']+)["']/g,
    /\b(?:src|href)="([^"]+)"/g,
    /new URL\("([^"]+)", import\.meta\.url\)/g,
];

test("every relative import, src and href in the app points at a file", () => {
    const missing = [];
    for (const file of files(APP)) {
        const text = fs.readFileSync(file, "utf8");
        for (const re of PATTERNS) {
            for (const [, ref] of text.matchAll(re)) {
                if (/^(https?:|#|\$\{)/.test(ref) || ref === "") continue;
                assert.ok(!ref.startsWith("/"), `${path.relative(APP, file)}: absolute path ${ref} breaks under /app/`);
                let target = path.resolve(path.dirname(file), ref.split(/[?#]/)[0]);
                if (ref.endsWith("/")) target = path.join(target, "index.html");
                if (!fs.existsSync(target)) missing.push(`${path.relative(APP, file)} -> ${ref}`);
            }
        }
    }
    assert.deepEqual(missing, []);
});

test("nav and home links resolve from the app root", () => {
    const missing = [];
    for (const file of ["lib/ui.js", "home.js"]) {
        const text = fs.readFileSync(path.join(APP, file), "utf8");
        for (const [, ref] of text.matchAll(/href: "([^"]*)"/g)) {
            if (!fs.existsSync(path.join(APP, ref, "index.html"))) missing.push(`${file} -> ${ref}`);
        }
    }
    assert.deepEqual(missing, []);
});
