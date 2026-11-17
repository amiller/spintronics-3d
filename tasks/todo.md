# Spintronics: notes, references, self-contained sim

Third-party source: ~/installing/spintronics/ (upperstory/spintronics-simulator, ctfhacker/spintronics-rs)

## 0. Try existing (done)
- [x] Upper Story simulator runs locally from `public/` as static files (python http.server). Phaser 3 + planck.js (Box2D). 2D, ~5k lines of our-relevant JS. CC BY-NC-SA 4.0.
- [x] Hosted examples: simulator.spintronics.com/?linkID=457 (parallel), 459 (sine osc), 460 (ring osc), 461 (pulse gen)
- [x] spintronics-rs: Rust API that emits simulator save files
- [ ] Search for any other sims (3D, fan, YouTube-linked), community.spintronics.com

## 1. Notes (notes/)
- [ ] how-it-works.md: per-part mapping (chain speed=current, tension=voltage, resistor=damper, capacitor=spring, inductor=flywheel, junction=differential, transistor, diode=ratchet), with sources
- [ ] references.md: Steve Mould video, Upper Story docs/puzzle books, hydraulic/mechanical analogies literature (Maxwell, Firestone mobility analogy vs impedance analogy), Hackaday/EEJournal
- [ ] read simulator source to extract the actual part equations it uses

## 2. Own sim (sim/)
- [ ] single self-contained HTML, no build
- [ ] circuit solved as ODE (not rigid-body physics), rendered as sprockets + chain
- [ ] parts: battery, resistor, capacitor, inductor, junction, diode, transistor, AC source
- [ ] scope trace of speed/tension; examples: RC, LC oscillator, bridge rectifier

## Parallel work on fractal
- codex: not installed on fractal; zai via claude CLI w/ ANTHROPIC_BASE_URL? (to check)

## Status 2026-11-17
- [x] notes/references.md (+json): 52 refs, arXiv ids and paper URLs spot-checked
- [x] notes/how-it-works.md: all parts, sim equations w/ file:line, four examples explained (example JSON not fetched)
- [x] sim/codex: 3D circuit bench, 7 presets, 7/7 self-tests pass in Chrome — chosen base
- [x] sim/glm: works, 10/10 tests but looser and cruder visuals — not used

## 3. Mechanisms from first principles (sim/mech/<name>/, codex on fractal)
Each: model.js (port API) + bench.mjs (V–I, step, impedance sweep, fitted element) + index.html (3D cutaway) + REPORT.md
- [x] damper: silicone-oil resistor (shear-thinning) vs eddy-current alternative — oil Ω falls 70→43 with voltage (50 Ω part); eddy linear
- [x] differential: planetary junction, inertia/backlash parasitics — KCL 1e-17, backlash 377 µm; TRANSIENT NOT dt-CONVERGED
- [x] gyrator: gyroscope two-port; gyrator+C = synthetic inductor — Z12=-Z21 flat; synthetic L within 1.5% of flywheel, 3.25 Hz gimbal resonance
- [x] transistor: stiction brake (kit) vs flyball-governor BJT-like alternative — stiction hysteresis 1.515/1.165 N; governor β≈-1.6
- [x] chain: discrete-link chain, spiral spring, flywheel bearings, inerter alternative — LC 0.6214 vs 0.6195 Hz ideal, Q≈16, chordal ripple 1.2–2.9%; worker relaxed a failing taut-chain assertion
- [ ] review each: run bench, open page, check sources; then integrate into sim/codex as switchable "ideal / mechanical" parts
