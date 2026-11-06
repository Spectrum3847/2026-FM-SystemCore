# Params

Tuned values that should survive a deploy live in JSON, not Java: `src/main/deploy/params/<group>.json`,
optionally `params/robots/<robot>/<group>.json` (the robot is the config class, `FM2026` by default),
then the robot's own `/home/systemcore/params/<robot>/<group>.json`, which a deploy does not wipe.
Each value is declared in code (`spectrumLib/params`), and the log records what it was and which file
it came from under `Params/`. A bad file raises an alert and is skipped. The groups so far:

- `cameras`: Limelight rewind lengths, the settings resend period and
  `pushLimelightMounts` (`CameraParams`).
- `swerve`, per robot in `params/robots/<robot>/swerve.json`: the four CANcoder offsets
  (`SwerveOffsetParams`), within ±0.5 rotations. The defaults are the robot config's
  (`FM2026` and so on), and the files hold the same numbers (`SwerveOffsetParamsTest`). Read once at
  boot: a live change raises an alert and needs a save and a reboot.

With `Constants.TUNING_MODE` on, values can be changed live under `/Tuning/` and saved to the robot
with `/Tuning/SaveParams` while disabled; CI fails a PR to `main` that leaves it on. A live change
applies only while disabled with no FMS attached, so nothing changed in the disabled gap between
auto and teleop runs in teleop unchecked. A change made on the field waits (`Params/PendingLive`) and
applies once the robot is disabled off the field, unless it is changed back first.
`FM_PARAMS_LOCAL=<folder>` gives the simulator a local folder to save to (it uses `<folder>/<robot>/`).

The `swerve` and `soft-limits` files in the robot's folder are hard stops, so each carries a
`$hardware` stamp: the swerve CANcoder magnet offsets, which the CANcoders hold in their own flash
and which differ from chassis to chassis. They are read at boot, before the drivetrain writes this
boot's offsets, and logged under `Params/Hardware/SwerveMagnetOffsets/`. A save records the offsets
the robot booted with and the ones being saved. At boot the file is used only if the CANcoders read
one of those sets (to 0.001 rotations); otherwise, or if a CANcoder doesn't answer, the file is
refused with a HIGH alert, `Params/Files` shows `refused: ...`, and the values come from git. So a
SystemCore moved to another chassis doesn't bring the old chassis's offsets or soft limits along.
Save again to re-stamp. Files left in the old flat folder (`/home/systemcore/params/<group>.json`)
are read until the next save, which writes the robot folder's file and renames the old one
`.moved`; a flat-folder `swerve` or `soft-limits` file has no stamp, so it is refused (and renamed
`.refused` on save). On a robot last deployed with the flat folder, check
`/home/systemcore/params/*.json` first, and re-save or copy those values to git.

The `soft-limits` group (`params/soft-limits.json`) holds the hood's and the intake extension's
soft limits (`HoodConfig`, `IntakeExtensionConfig`; the right axis shares the left's). The defaults
are the code's and the file holds the same numbers; each param's bounds keep every position the code
writes into the motor between the two limits (`SoftLimitParamsTest`). A mechanism reads them once,
when it is built: a live change raises an alert and needs a save and a reboot.
