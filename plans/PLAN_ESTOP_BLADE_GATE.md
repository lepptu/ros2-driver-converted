# Plan: E-stop must stop the blade for EVERY commander (arduino_bridge gate)

> Status: **IMPLEMENTED 2026-07-08 (bench, awaiting field test)** — code in
> `scripts/arduino_bridge.py`. Field-found 2026-07-08. Latch state-machine
> unit-simulated for all four test-plan scenarios (gamepad held-through,
> web/mission instant clear, watchdog fallback, no-blade e-stop). Deploy:
> `mowbot-launch-bringup` restart (symlink-installed, no rebuild). **User owes
> the field test below + the bringup restart.**
>
> Location rationale: the fix lives in `scripts/arduino_bridge.py` in THIS
> repo — the single choke point every blade command already passes through.
>
> Companion docs: `mowbot_web_ui_remote/plans/implemented/
> PLAN_MANUAL_BLADE_CONTROL.md` (the web-UI manual blade feature whose
> field test exposed this), `mowbot_web_ui_remote/plans/
> PLAN_ESTOP_MOTION_GATE.md` (the sibling find: e-stop doesn't stop manual
> *driving* either — separate mechanism, twist_mux), `mowing_navigation/
> plans/FIELD_TESTING.md` **F30**, BT_REVIEW §7 Phase 0 (Arduino firmware
> watchdog — the related hardware-safety debt).

## Field finding (2026-07-08)

Pressing the physical e-stop while the blade is commanded from the
**gamepad** does **not** stop the blade. It keeps spinning until the button
is released.

## Why it happens — e-stop is a *signal*, not a power cut

The e-stop reaches the blade only as data: Arduino reads the switch and
reports it in its serial status frame; `arduino_bridge` publishes it as
`/eStop_status`. Nothing in the Arduino→motor path enforces it. Every stop
the field has ever seen was **software downstream of that topic**:

| Blade commander | E-stop enforcement today | Field result |
|---|---|---|
| Mowing mission (`MowMotorController` + BT) | HoldBranch: `IsEstopClear` fails → blade off (BT_REVIEW S3, field-confirmed round 1) | stops ✓ |
| Web UI manual mode (bridge `manual_mow` manager) | manager subscribes `eStop_status` → refuses/auto-offs (field-confirmed 2026-07-08) | stops ✓ |
| **Gamepad (`joy_to_arduino`)** | **nothing** — keep-alive keeps re-enabling while the button is held; `arduino_bridge` relays it | **keeps spinning ✗** |

`arduino_bridge` itself *sees* the e-stop state on every serial frame
(`parts[0]` of the status line) but only republishes it — it is never
stored or acted on. Line-level facts in `scripts/arduino_bridge.py`:
`self.eStop_state` is declared in `__init__` but the read loop
(`read_from_arduino`) builds the `Bool` message straight from `parts[0]`
without updating it; `send_to_arduino` computes `effective_en` from the
watchdog + RPM guard only.

## Fix: gate the blade on e-stop inside `arduino_bridge`

One gate at the choke point covers the gamepad, the web UI manager, the
mission, and any future commander — defense in depth even for the paths
that already stop themselves.

### Changes (`scripts/arduino_bridge.py`, ~15 lines)

1. **Store the state**: in `read_from_arduino`, set
   `self.eStop_state = int(parts[0])` alongside the existing publish.
2. **Gate in `send_to_arduino`** (next to the existing watchdog + RPM
   guard):
   - while `eStop_state == 1` → force `effective_en = 0`, log once
     (edge-triggered, English, matching the watchdog log style:
     `"mowMotorEN e-stop gate: e-stop active — blade blocked"`).
3. **No auto-resume latch**: on the e-stop **release** edge, keep the gate
   latched until the commander has been seen to drop enable
   (`mowMotorEN_cmd data: false` arrives, or the keep-alive watchdog
   trips). Rationale: a gamepad button still held through an e-stop cycle
   must NOT restart the blade by itself — the operator must release and
   press again. The mission and the web-UI manager already publish
   `false` on e-stop (their own handling), so their latch clears
   instantly and their existing resume flows are unchanged.
4. Log the unlatch (`"...e-stop gate: cleared — enable released"`).

### Deliberately NOT in scope

- **Movement** on e-stop — separate mechanism (twist_mux lock), separate
  plan: `mowbot_web_ui_remote/plans/PLAN_ESTOP_MOTION_GATE.md` (F31).
- `joy_to_arduino` changes — pointless once the bridge gates; hold-to-run
  stays as is.
- Removing the software handling from the mission/web-UI paths — they stay
  as first-line handling (better UX: their status topics explain *why*);
  the bridge gate is the backstop.

### Hardware/firmware follow-up (recommended, separate task)

The real lesson is that **blade-off currently depends on the Pi**: if the
Pi (or `arduino_bridge`) freezes with the blade on and enable latched high
on the serial line, only the serial-silence behavior of the firmware saves
us. The Arduino firmware should enforce e-stop → motor-enable LOW in
firmware (it reads the switch directly), independent of anything the Pi
sends. This folds into the existing "Arduino firmware blade watchdog"
hardware-safety debt (BT_REVIEW §7 Phase 0 / FIELD_TESTING status board).
Ideally the e-stop circuit would also hard-cut blade motor power — worth
checking whether the wiring allows it.

## Deployment

`arduino_bridge.py` is symlink-installed → **no rebuild**, but it runs
inside **bringup** → `mowbot-launch-bringup` restart required (do it
between missions; F12's `KillMode=control-group` now makes stops clean).

## Test plan (field, blade area clear)

1. Gamepad: hold blade button → blade on → **press e-stop → blade stops
   ≤0.1 s** (next 10 Hz serial frame) while the button stays held.
2. Keep holding through e-stop release → blade must **stay off**; release
   the button, press again → blade runs (latch cleared by the release
   edge's `false`).
3. Web UI manual mode: unchanged behavior (manager still auto-offs with
   `reason:"estop"`; re-engage after release works — the manager's `false`
   cleared the latch).
4. Mission: e-stop mid-mow → blade off + SAFETY_HOLD exactly as in round
   1; resume flow unchanged.
5. Journal shows the gate/unlatch log lines with the right edges.
