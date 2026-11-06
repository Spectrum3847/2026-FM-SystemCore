# Controls and the Driver Station

## Bindings

Pilot (port 0):

| Input | Does |
|---|---|
| RT | Launch (squeeze after 1 s); RT + LT launch without squeeze; LT released with RT held, launch with no delay. All behind the shot gate |
| LT | Intake; LT + LB eject |
| X (without LB) | Track target: aim and spin up, no feed |
| A (without LB) | Unjam |
| **LB + A / LB + X / LB + B** | **Set shot: tower / left trench / right trench** |
| Select | Force home |
| LB + D-pad | Reorient forward / left / back / right |
| A / B, disabled | Coast / brake the intake extension and hood |

Operator (port 1):

| Input | Does |
|---|---|
| **Y (without LB)** | **Hold to feed regardless of the shot gate** |
| LB + Y | Intake extension: reset position to max |
| Select | Force home |
| Start | Release mechanisms held by their tracking stop (off by default, see `Mechanism.Config`) |
| D-pad up / down | Hood trim +/- 0.1 deg |
| D-pad left / right | Aim trim +/- 1 deg |
| A / B, disabled | Coast / brake |

The chords don't depend on press or release order (`ModifierChords` in spectrumLib). A or X pressed
or let go within 0.08 s of LB counts as part of the chord, so bare unjam and track target start
0.08 s after the button goes down. A tap shorter than that still runs, 0.08 s late, for as long as
it was held. Letting go of LB with X or A still held goes IDLE, then to the bare action once it
has been held 0.08 s without LB.

These tables are also `BINDINGS` in [BindingsSmokeTest](../src/test/java/frc/robot/BindingsSmokeTest.java),
which presses every input in simulation. Change a binding in both.

## Controllers and the Driver Station

The October event runs the 2026 FMS, so FM is driven from the **2026 NI FRC Driver Station**. The
2027 WPILib DS is "not supported on any current FIRST FMS"
([FirstDriverStation-Public](https://github.com/wpilibsuite/FirstDriverStation-Public); the
[SystemcoreTesting](https://github.com/wpilibsuite/SystemcoreTesting) README table rates the NI
DS compatible with images 10-13 on alpha-5/6).

The two Driver Stations send an Xbox controller's data in different orders, and nothing between
the DS and robot code reorders it. So WPILib 2027's `Gamepad` / `CommandGamepad` reads the wrong controls on
the NI DS:

| Control | NI DS (index) | 2027 DS / `Gamepad` (index) | What `CommandGamepad` read on the NI DS |
|---|---|---|---|
| A B X Y | 0 1 2 3 | 0 1 2 3 | right |
| LB, RB | 4, 5 | 9, 10 | RS, nothing |
| Back, Start | 6, 7 | 4, 6 | LB, Back |
| LS, RS | 8, 9 | 7, 8 | Start, LS |
| D-pad | POV 0 | buttons 11-14 | left and right only (the port also read the POV), no up or down |
| LX, LY | axes 0, 1 | 0, 1 | right |
| RX, RY | axes 4, 5 | 2, 3 | LT, RT |
| LT, RT | axes 2, 3 (0 to 1) | 4, 5 | RX, RY |

[GamepadLayout](../src/main/java/frc/spectrumLib/gamepads/GamepadLayout.java) holds both tables, and
[Gamepad](../src/main/java/frc/spectrumLib/gamepads/Gamepad.java) reads raw indices through one of
them. Pilot and Operator bindings did not change. The layout is picked by `Gamepad.Config.mapping`:

- **`NI_DS`** is the default, and is what the event needs (the FMS runs the 2026 NI DS).
- **`WPILIB_DS`** is for driving from the 2027 DS.
- **`AUTO`** guesses from what the DS says the controller has. It needs all six axes. A POV means
  the NI layout, even with button 14 present, and button 14 with no POV means the 2027 layout. If
  it can't tell, it keeps the last layout it recognised, starting from NI. It is off by default, because the guess is ours and not
  documented anywhere.

Whatever the mapping, the same guess raises a warning alert when the controller looks like the
other layout ("Pilot controller looks like the WPILIB_DS layout, but is read as NI_DS").

**Checked against sources:**
- The SystemCore HAL copies MrcCommDaemon's joystick arrays without reordering them
  (allwpilib `v2027.0.0-alpha-6`, `hal/src/main/native/systemcore/FIRSTDriverStation.cpp`).
- WPILib ships `NiDs*` controller classes because "the old DS doesn't have the same mappings as a
  new DS" ([allwpilib#8376](https://github.com/wpilibsuite/allwpilib/pull/8376)), and because
  "FRC teams need the old classes" until the new DS works with the FMS
  ([SystemcoreTesting#322](https://github.com/wpilibsuite/SystemcoreTesting/issues/322)).
- The NI table is alpha-6's `NiDsXboxController`, checked by
  [GamepadLayoutTest](../src/test/java/frc/spectrumLib/gamepads/GamepadLayoutTest.java).
- Team 10183 hit this at an event on alpha-6 with the NI DS. They moved to raw axes 2/3/4 and
  raw button 6 for Back
  ([Summer-Scorcher](https://github.com/arip613/Summer-Scorcher/blob/main/src/main/java/frc/robot/Robot.java)).
- AdvantageKit logs and replays each joystick's name, type, and available buttons, axes and POVs,
  so the mapping and the `AUTO` guess replay.
- The sim GUI's "Map gamepad" (Windows) produces the NI order too.

**Not verified; needs the bench test below:**
- MrcCommDaemon is closed source, so the NI order on alpha-6 / image alpha13 rests on the sources
  above rather than on a run on our robot.
- What the NI DS reports as available buttons and POVs (and so whether `AUTO` guesses right).
- Trigger range and rumble through the NI DS.
- Team 302's note that `CommandGamepad` "should work with both" contradicts all of the above and
  was not tested.

### Bench test (NI DS, about 5 minutes)

1. Plug an Xbox controller into the laptop running the **2026 NI Driver Station**. Put it on USB
   slot 0, then 1 for the operator. Connect to the robot. The robot can stay disabled.
2. Open Elastic or AdvantageScope on `/AdvantageKit/RealOutputs/Gamepads/Pilot/` (or
   `./gradlew ntDump -Phost=172.26.0.1 -Pprefix=/AdvantageKit/RealOutputs/Gamepads`).
3. Check `Mapping` = `NI_DS`, `Layout` = `NI_DS`, `DetectedLayout`, `Name` and
   `Raw/ButtonsAvailable` (expect 1023 = 10 buttons), and `Raw/POV` = `CENTER`. Write down
   `DetectedLayout`: it tells us whether `AUTO` would work.
4. Press each control in turn: A, B, X, Y, LB, RB, LT, RT, Back, Start, LS, RS, D-pad up, down,
   left, right. **`Pressed` must name exactly that control.** `Raw/Buttons` shows the bit, and
   `Raw/POV` the D-pad.
5. Move each stick to its limits and pull each trigger:
   - `LeftX` / `RightX` go +1 to the right.
   - `LeftY` / `RightY` go −1 when pushed up.
   - `LeftTrigger` / `RightTrigger` go 0 to +1.
   - The `Raw/Axis0-5` values show where each one really arrives.
6. Repeat on the operator controller (`Gamepads/Operator/`).
7. If anything is off, the `Raw/*` values give the real indices. Fix the `NI_DS` row in
   `GamepadLayout` (and the test) and redeploy.
