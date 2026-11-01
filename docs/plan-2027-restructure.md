# 2027 Restructure Plan

Branch: `2027-Restructure`. This repo (`2026-FM-SystemCore`) is the prototype for the 2027
workflow: robot app, tuning/params system, logging, replay, and AI-assisted development. This plan
turns that prototype into the 2027 repo layout.

## Goals

- Separate the reusable framework (`spectrumLib`) from robot-specific code into its own repo.
- Give each robot (or robot family) its own repo, with shared code via the library.
- Make the structure and naming AI-friendly and stable before the 2027 repos are created.
- Rename things we do not like (notably "Params") before they spread into new repos.

## 1. Repo split

### 1.1 Shared library: SpectrumLib

A new repo, `SpectrumLib`, owns the reusable framework currently under
`src/main/java/frc/spectrumLib/` plus the generated-contract machinery and the tuning system.

Contents:
- `mechanism`, `telemetry`, `hardware` (CAN buses, config budget, Rio), `util`, `sim`
- `localization`, `vision` helpers, `checks`, `gamepads`, `leds`
- The tuning system (see naming in section 3)
- The robot-app generated contract (params schema, manifest fields, bindings, log-key freshness)
- Common tests and sim harnesses

Delivery: published as a versioned Maven artifact (GitHub Packages or an internal Maven repo).
Robot repos depend on it by version. For local development, a composite build
(`includeBuild`) lets a developer work against an unreleased library commit.

### 1.2 Robot repos

| Robot / family | Repo | Notes |
| --- | --- | --- |
| Photon | `2027-Photon` | Own repo; own SpectrumLib version. |
| Rebuilds / new robots | `2027-<name>` | Own repo each. |
| Offseason robots | `2027-<name>-offseason` (or a shared offseason repo) | Own repo; may track a stable SpectrumLib. |
| AM | `2027-AM` | Own repo; very different from the others. |
| XM and PM | `2027-XM` (shared) | Functionally identical; build-quality differences only. |

XM/PM sharing: one repo, one codebase. Build-quality differences live in config/constants
(`XMConfig` vs `PMConfig`, selected by a Gradle property or a small config switch), not in
separate code paths. Deploy targets differ by robot identity.

### 1.3 What stays in a robot repo

- `Constants`, `Robot`, `Event`, robot identity
- Mechanism implementations and their constants (gear ratios, CAN IDs, offsets, limits)
- `VisionConfig`, camera mounts, field/event selection
- Autos, bindings, SuperStructure
- `src/main/deploy/app/` pages specific to that robot (shared infra comes from SpectrumLib)
- `src/main/deploy/params/*.json` (tuned values)
- `docs/` for that robot

### 1.4 Migration path

1. Extract `spectrumLib` as a Gradle module inside this repo (composite build) and prove it
   compiles and tests pass with the robot module depending on it.
2. Publish the module to Maven local; switch the robot module to consume the artifact.
3. Move the module to the `SpectrumLib` repo; set up CI to publish on tag.
4. Create robot repo templates from this repo's robot side, parameterized by robot name and config.
5. Move Photon first (simplest), then AM, then XM/PM, then offseason/rebuilds.

### 1.5 Versioning policy

- SpectrumLib uses SemVer, with the supported WPILib version in the release notes.
- Robot repos pin a SpectrumLib version; bumps are deliberate PRs.
- During an event, SpectrumLib bug fixes are allowed (smallest fix, per AGENTS.md) but become a
  PR plus a version bump; robot repos then bump.

## 2. Repo structure for 2027 (AI-heavy development)

This repo is the prototype. The 2027 layout should make it easy for an AI session to understand,
change, and verify the robot safely.

### 2.1 Principles

- **Single source of truth.** Keys, params, bindings, manifest fields, and log keys are generated
  from Java (the generated contract). Nothing is typed by hand in a page or doc.
- **Strong, fast CI.** Build, unit tests, sim tests, scripted sims, app tests (`tsc --noEmit`),
  log-key freshness, and replay (when fixtures exist). A PR that fails CI does not merge.
- **Clear module boundaries.** Library vs. robot. The library is versioned; the robot is the
  application.
- **Colocated context.** `AGENTS.md`, `CLAUDE.md`, and a short `docs/architecture.md` per repo
  describe conventions, safety rules, and how to run/verify. Keep them accurate; update them in the
  same PR as the change.
- **Safety is explicit.** Hard-stop values (CAN IDs, inversions, offsets, current limits, soft
  limits, gear ratios, brake/coast, zero positions) require human confirmation and a PR block.
  CI cannot catch hardware changes, so the PR template carries the checklist.
- **Tuning is data, not code.** Tuned values live in JSON, applied only while disabled, logged as
  inputs, and pulled back to git via the tuning workflow. AI may propose values; hard-stops need
  confirmation.
- **Replay is the oracle.** Robot behavior changes are verified by sim, replay, or a described
  bench test — never by "it compiles."

### 2.2 Proposed robot repo layout

```
2027-<robot>/
  build.gradle
  settings.gradle
  AGENTS.md
  CLAUDE.md
  docs/
    architecture.md
    log-keys.md
  src/main/java/frc/
    robot/          # robot-specific
    # spectrumLib comes from the dependency, not this tree
  src/main/deploy/
    app/            # pages + generated contract
    params/         # tuned values (JSON)
  src/test/...
  tools/            # check.py, replay helpers, app tests
```

### 2.3 AI workflow

- One feature per PR; small, reviewable changes.
- Generated files are regenerated, never hand-edited.
- A PR that renames a log key, NT key, param, or binding updates the page that reads it.
- Hard-stop changes include old value, new value, and failure mode in the PR.
- Verification is stated under "How I verified" (sim, replay, or bench test).

## 3. Naming changes

"Params" is not a favorite name and should be renamed before the 2027 repos are created, since the
rename touches code, NT keys, log keys, docs, and app pages.

### 3.1 Candidate names

| Current | Candidate | NT prefix | Files |
| --- | --- | --- | --- |
| Params | Parameters | `/Parameters/` | `parameters/*.json` |
| Params | Settings | `/Settings/` | `settings/*.json` |
| Params | Calibration | `/Calibration/` | `calibration/*.json` |

Recommendation: **Parameters** (the full word, not "Params"). It is neutral, does not collide with
WPILib's `org.wpilib.tunable.Tunable`/`Tunables` (which would make "Tunables" confusing), and
does not collide with `Config` (robot configuration) or WPILib's `Preferences`. "Settings" is an
acceptable alternative if "Parameters" still feels too generic.

### 3.2 Rename scope (if Tunables is chosen)

- Package: `frc.spectrumLib.params` → `frc.spectrumLib.parameters`
- Classes: `Params`, `ParamsInputs`, `Params.group(...)` → `Parameters`, `ParametersInputs`, `Parameters.group(...)`
- NT topics: `/Params/...` → `/Parameters/...`; `/Tuning/SaveParams` → `/Tuning/SaveParameters`
  (or keep `/Tuning/` as the action namespace)
- Log keys: `/Params/Sources/<path>` → `/Parameters/Sources/<path>`
- Files: `src/main/deploy/params/` → `src/main/deploy/parameters/`; `params/robots/...` accordingly
- Gradle tasks: `pullTuning` stays (it pulls tuned values); `fetchRobotParams`/`applyRobotParams`
  → `fetchRobotParameters`/`applyRobotParameters`
- App: `Tuning` page labels and the generated module names
- Docs: `docs/robot-app-plan.md`, `AGENTS.md`, `README.md`, `docs/log-keys.md`
- Tests and `tools/` that reference params paths

### 3.3 Other naming to settle before 2027

- Library repo/package: keep `spectrumLib` / `frc.spectrumLib` (familiar; no need to churn).
- Robot app: "Robot App" is the settled name (per AGENTS.md). Keep it.
- `LoggedNetworkChooser` (already renamed from `LoggedDashboardChooser`): keep.
- Branch prefixes: keep `event*`, `claude/*`, `upgrade/*`; add `restructure/*` if needed.
- Robot names: confirm XM/PM/AM/Photon naming is final before creating repos.

## 4. Open questions

1. Composite build vs. published artifact for local dev: do we require network access to build a
   robot, or allow `includeBuild` overrides?
2. XM/PM: one repo with a config switch, or two repos sharing a template? Recommend one repo.
3. Does the Robot App ship inside SpectrumLib or per robot? Recommend per robot, infra in library.
4. Parameters rename: confirm "Parameters" over "Settings"/"Calibration" before executing (note:
   WPILib already uses `Tunable`/`Tunables`, so that name is avoided).
5. Which robots get their own repo vs. sharing a template repo?

## 5. Checklist

- [ ] Extract `spectrumLib` as a Gradle module in this repo and prove the build.
- [ ] Publish to Maven local; switch robot module to consume it.
- [ ] Create `SpectrumLib` repo and CI publish.
- [ ] Create robot repo templates.
- [ ] Migrate Photon, then AM, then XM/PM.
- [ ] Decide and execute the Parameters rename.
- [ ] Update AGENTS.md/CLAUDE.md/docs for the new layout.
