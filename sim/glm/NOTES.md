# sim/glm — 3D Spintronics-style circuit simulator

Single self-contained `index.html` (three.js r0.160.0 via CDN importmap, no build step, no other deps).
Physics is circuit-level, not rigid-body: chain speed ↔ current, chain tension ↔ voltage.

## Solver design

- **MNA** (modified nodal analysis), dense Gaussian elimination with partial pivoting.
  Unknowns: node voltages + branch currents of voltage sources.
- **Trapezoidal companion models** for C and L (Norton form):
  - Capacitor: `i₁ = G·v₁ − (i₀ + G·v₀)`, `G = 2C/h`
  - Inductor: `i₁ = G·v₁ + (i₀ + G·v₀)`, `G = h/(2L)`
  - First step after an element is created uses backward Euler for consistent
    initial conditions (otherwise trapezoidal rings on inconsistent states).
- **Step size** `h = 1 ms`, up to 60 steps per animation frame.
- **Diode** (ratchet): exponential `I = Is(e^(vd/Vt)−1) + gmin·vd` with
  `Is = 2e-10, Vt = 0.02, gmin = 1e-9`, exponent clamped at `Vt·80`;
  linearized by iterated stamps (Newton) until converged.
- **Transistor**: voltage-controlled resistor, matching the official sim's
  damping law: `R(r) = max(0.5, 100^(2(1−smoothstep(r))) − 1)` where `r` is the
  gate travel fraction (gate limits 6.48°–45°, per transistor-part.js:6-7,289-293
  of the official source). Gate draws no current. Solved in the same iterated-stamp loop.
- **Switch**: conductance `1e4` on / `1e-9` off (stamped and committed identically).
- **Ammeter**: 1 mΩ branch, read via branch current.
- **Drag torque**: click-drag on a sprocket injects a current source
  `I = clamp(ω·0.8, ±8)` at that node (equivalent to the official Interact tool's torque).
- **AC source** (crank): sinusoidal voltage source. Not an official Spintronics part;
  added for the rectifier preset.

Part-model mapping (verified against the official simulator source, which is
CC BY-NC-SA 4.0 — read for reference only, no code or assets copied):

| Part | Official physics | This sim |
|---|---|---|
| Resistor | angular damping = R/5 | resistance R |
| Capacitor | spring torque ∝ 1/C | capacitance C |
| Inductor | density (inertia) ∝ L | inductance L |
| Battery/motor | constant torque | constant voltage |
| Transistor | damping = 100^(2(1−r))−1, min 0.5 | R(r) same formula |
| Diode | ratchet (one-way) | exponential diode |

## Tests (`node test.mjs`, 10/10 pass, ~0.8 s)

`test.mjs` regex-extracts the `SOLVER BEGIN…END` section from `index.html`
and imports it as a module, so the tested code is byte-identical to what the
browser runs. Same tests also run in-browser with `?test=1`.

| Preset | Measured | Analytic | Pass |
|---|---|---|---|
| RC time constant | 1.000 s | 1.000 s | ✓ |
| RL time constant | 1.001 s | 1.000 s | ✓ |
| LC period | 6.2832 s | 6.2832 s | ✓ |
| LC amplitude (7 cycles) | 5.987 V | 6 V | ✓ |
| Rectifier V(out) min/peak | 4.63 / 5.15 V | ripple small, peak < 6 | ✓ |
| Rectifier ripple | 0.25 V | — | ✓ |
| Ring oscillator | 9 rising crossings of V(s0) | oscillates | ✓ |
| Parallel resistors | I = 1.05000 | 1.05 exact | ✓ |
| Amplifier | gain 3.40×, inverting | ~2.5×, inverting | ✓ |

## UI verification

Headless chromium (playwright): all 7 presets load with zero page errors;
scene graph populated (35 meshes, 30 chain-link instances, valid instance
matrices); scope canvas contains red/blue trace pixels at exactly `#f66`/`#6cf`
plus numeric range labels. `?test=1` prints 10/10 PASS to the console.

## Known gaps

- Ring oscillator swing is 0.15–4.37 V, not full 0–6 V rail-to-rail.
- Amplifier gain is 3.40× vs the ~2.5× design target (still correctly inverting).
- Transistor gate is a static smoothstep voltage→resistance map, not the
  official gate-spring dynamics (gate angle limits are honored in the visual only).
- Drag torque is a current injection at a node, not a true series voltage source,
  so it can't model torque applied across a single element.
- AC source is a non-official addition (flagged in the UI).
- Visual layer verified by headless smoke test + pixel assertions, not interactively.
