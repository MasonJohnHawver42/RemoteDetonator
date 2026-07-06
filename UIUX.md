# Master Unit — UI/UX Screen Reference

## Hardware Controls

| Button      | GPIO | Behavior         |
|-------------|------|------------------|
| BTN_CYCLE   | 42   | Active-low, cycle/select |
| BTN_ADVANCE | 40   | Active-low, advance/fire |

## Display Layout (128×64 OLED)

Every screen shares the same chrome:

```
[ MASTER ]                 NN%   ← fixed header (battery top-right, refreshes every 2s)
────────────────────────────────
Line 1  (y=32)
Line 2  (y=48)
```

---

## Screens

### 1. INIT — Title / Press-to-start

Shown on boot (after a 2 s radio warmup).

```
[ MASTER ]                  NN%
                                
Remote Det
press a button
```

**Navigation:** Any button **release** (either BTN_CYCLE or BTN_ADVANCE) → **STANDBY**

Note: while on INIT the device silently continues watching for a pending fire response.
If one arrives (or times out), Line 2 changes to one of the error messages below before the operator presses anything.

---

### 2. INIT — Error / Status message

Same layout as Title, but Line 2 carries an error string.  
Reached automatically (no button press) from any of the error conditions listed below.

```
[ MASTER ]                  NN%
                                
Remote Det
<message>
```

| `<message>`      | Cause                                                        |
|------------------|--------------------------------------------------------------|
| `radio fail`     | Radio hardware failed to initialise                         |
| `arm tx error`   | LML rejected the Arm packet during TX                       |
| `fire tx error`  | LML rejected the Fire packet during TX                      |
| `arm link error` | LML reported a link error while waiting for Armed response  |
| `arm timeout`    | No Armed response received within 10 s                      |
| `bad arm resp`   | Armed response contained wrong slot/device/action_id        |
| `already fired`  | Slave replied Disarmed (slot already consumed)              |
| `fire timeout`   | No Fired response received within 10 s of Fire being sent   |
| `bad fire resp`  | Fired response contained wrong slot/device/action_id        |
| `MISFIRE`        | Slave replied Misfired                                       |
| `reset slot N`   | Operator triggered a 1-tap reset for the current slot       |
| `reset dev N`    | Operator triggered a 2-tap reset for the current device     |
| `reset all devs` | Operator triggered a 3-tap reset for every device           |

**Navigation:** Any button **release** → **STANDBY**

---

### 3. STANDBY — Pair selected

Idle screen. Displays the currently selected slot/device pair.

```
[ MASTER ]                  NN%
                                
STANDBY
slot N dev M
```

**Navigation from STANDBY:**

| Gesture                          | Result                                                    |
|----------------------------------|-----------------------------------------------------------|
| Release BTN_CYCLE (no chord)     | Advance to the next slot/device pair (wraps around); Line 2 updates in place |
| Release BTN_ADVANCE (plain tap)  | → **OPERATIONAL / Arming**                               |
| Hold BTN_CYCLE + tap BTN_ADVANCE 1× then release BTN_CYCLE | → **INIT** (`reset slot N`) — resets the current slot on the current device |
| Hold BTN_CYCLE + tap BTN_ADVANCE 2× then release BTN_CYCLE | → **INIT** (`reset dev N`) — resets all slots on the current device |
| Hold BTN_CYCLE + tap BTN_ADVANCE 3× (or more) then release BTN_CYCLE | → **INIT** (`reset all devs`) — resets every device listed in PAIRS |

---

### 4. OPERATIONAL / Arming

Entered immediately when BTN_ADVANCE is released in STANDBY.  
An Arm packet is sent to the slave; the screen waits up to 10 s for an Armed response.

```
[ MASTER ]                  NN%
                                
Arming
slot N dev M
```

**No button input is processed in this state** (button events are ignored while waiting for the Armed response).

Auto-transitions (no button press):

| Condition                   | Next screen                        |
|-----------------------------|------------------------------------|
| Armed response OK           | → **OPERATIONAL / Firing** (immediate, no screen pause) |
| Armed response wrong fields | → **INIT** (`bad arm resp`)        |
| Disarmed response           | → **INIT** (`already fired`)       |
| Any other response          | → **INIT** (`bad arm resp`)        |
| LML link error              | → **INIT** (`arm link error`)      |
| 10 s timeout                | → **INIT** (`arm timeout`)         |

---

### 5. OPERATIONAL / Firing

Reached automatically from Arming when the Armed response is verified.  
A Fire packet is sent immediately; the screen is visible only for the round-trip radio time before STANDBY is re-entered.

```
[ MASTER ]                  NN%
                                
Firing
slot N dev M
```

**No button input is processed in this state.**

Auto-transition (no button press):

| Condition           | Next screen                |
|---------------------|----------------------------|
| Fire packet queued OK | → **STANDBY** (immediately; background watcher starts) |
| LML TX error        | → **INIT** (`fire tx error`) |

After returning to STANDBY, the background watcher runs silently (no visible change) until a Fired/Misfired response arrives or the 10 s fire timeout expires, at which point the display reverts to one of the INIT error screens.

---

## Full State Flow Diagram

```
Boot
 └─► INIT "press a button"
       │
       │  any button release
       ▼
     STANDBY "slot N dev M"
       │  ▲
       │  │ BTN_CYCLE release (plain) ─────────────► cycle pair (stays in STANDBY)
       │  │
       │  │ BTN_CYCLE hold + BTN_ADVANCE tap(s) ───► INIT (reset message)
       │  │
       │  BTN_ADVANCE release (plain)
       ▼
  OPERATIONAL / Arming
       │  (10 s window)
       │
       ├── timeout / error ──────────────────────────► INIT (error message)
       │
       └── Armed OK
            ▼
       OPERATIONAL / Firing
            │
            ├── tx error ──────────────────────────────► INIT (error message)
            │
            └── Fire queued
                  │
                  ├──────────────────────────────────────► STANDBY (background watcher active)
                  │                                              │
                  │                    Fired OK (within 10 s) ──┤ (stays in STANDBY, silent)
                  │                    Misfired / bad / timeout ─► INIT (error message)
                  └─ (next operator cycle continues from STANDBY)
```
