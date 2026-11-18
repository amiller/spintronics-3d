# Spintronics Circuit Bench

Open `index.html` directly or serve this directory as static files. The interactive page loads three.js r160 and OrbitControls from jsDelivr. The inline solver and `?test=1` mode do not load three.js. [Import map](index.html#L8) · [Test entry](index.html#L512)

## Solver

- Modified nodal analysis solves node voltages and the currents through ideal sources and the ammeter. An oriented passive branch stamps `i = G(v + u) + j`, where `u` is the voltage applied by dragging a sprocket and `j` carries the integration history. The solver uses partial pivoting. [Linear solve](index.html#L231) · [Branch stamps](index.html#L299) · [Drag torque](index.html#L1296)
- The fixed step is 1/480 s. Capacitor charge and inductor current use trapezoidal integration in the linear presets, with backward Euler at startup and after discontinuous parameter or switch changes. The bridge, ring, and amplifier use backward Euler throughout. Each step checks the ratchets’ conduction states and the gates’ travel limits. The channel resistance uses the previous accepted gate travel. [Step size](index.html#L127) · [Preset methods](index.html#L156) · [Step and constraints](index.html#L350)
- Sprocket rotation uses integrated branch current scaled by inverse pitch radius. The gate’s charge and adjustable compliance set its travel. Chain instances follow exact circular arcs and tangent spans. Link color and span sag use the largest absolute branch voltage in each displayed loop. [Chain path](index.html#L829) · [Rotation and links](index.html#L883)

## Part references

- The spring motor supplies constant drive while its breaker is reset. The reference trips from motor speed. The equivalent model trips from branch current and adds an adjustable current threshold. The ratchet permits forward motion. [Motor source](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/motor-part.js#L263) · [Ratchet source](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/diode-part.js#L87) · [Breaker model](index.html#L363)
- The resistor maps angular damping to resistance, the flywheel maps inertia to inductance, and the capacitor maps restoring torque to charge divided by capacitance. [Resistor damping](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/resistor-part.js#L177) · [Flywheel density](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/inductor-part.js#L158) · [Capacitor torque](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/capacitor-part.js#L224)
- The gate spring uses the reference travel limits of 0.018–0.125 turns. If `r` is the fraction of that travel, the channel resistance is the slider scale times `max(0.5, 100 ** (2 * (1 - r)) - 1)`. The junction’s 3D differential shows the current split enforced by the electrical node. [Gate limits](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/transistor-part.js#L6) · [Gate torque and brake](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/transistor-part.js#L269) · [Junction gears](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/junction-part.js#L160) · [Circuit brake](index.html#L286) · [Differential model](index.html#L694)
- The AC crank is an original, non-official part. Its eccentric follows the same phase as its sinusoidal voltage. Drag interaction uses a series voltage corresponding to shaft torque. The reference interaction uses a mouse joint. [AC branch](index.html#L299) · [Procedural parts](index.html#L718) · [Reference interaction](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/part-manager.js#L785)

## Verification

`?test=1` prints the following checks in the console and on the page. I ran the inline solver with Node v24.22.1. [Assertions](index.html#L396)

| Preset | Simulated time | Result |
| --- | ---: | --- |
| RC | 6 s | PASS. Time constant 1.0003 s, analytic 1.0003 s |
| RL | 6 s | PASS. Time constant 0.2999 s, analytic 0.2999 s |
| LC | 6 s | PASS. Period 2.2214 s, analytic 2.2214 s |
| Bridge | 6 s | PASS. Output 0–4.4095 V, analytic ceiling 4.8128 V, minimum diode current 0 A |
| Ring | 12 s | PASS. Late swing 1.51 V per stage, eight rising crossings, growth 2.60 s⁻¹ versus analytic 2.44 s⁻¹ |
| Parallel | 6 s | PASS. Total 0.7500 A, analytic 0.7500 A |
| Amplifier | 6 s | PASS. Inverted gain magnitude 3.272, small-signal analytic 3.324 |

From the repository root, reproduce the solver checks with:

```sh
node --input-type=module <<'JS'
import {readFileSync} from 'node:fs';
const html = readFileSync('sim/codex/index.html', 'utf8');
const code = html.split('<script type="module" id="solver">')[1].split('</script>')[0];
const {runTests} = await import('data:text/javascript;base64,' + Buffer.from(code).toString('base64'));
if (runTests().some(test => !test.pass)) process.exitCode = 1;
JS
```

I also ran 53,760 steps with seeded parameter changes and drag inputs. The worst node-current residual was 7.69 × 10⁻¹¹ A. A Node smoke test using local three.js r177 with a stub DOM and renderer exercised all seven scene layouts, drag picking, slider edits, reset, and pause. Browser rendering with the pinned r160 CDN import is unverified. The sandbox blocked Chromium startup and CDN DNS resolution, and headless Firefox exited during startup.

## Known gaps

- Models use equivalent circuit values. Parasitic inertia, gate-dependent channel inertia, chain elasticity, friction, and backlash are omitted. An open switch or tripped breaker uses 1 MΩ. The bridge includes two symmetric 10 MΩ reference paths to define its floating AC common mode while the diodes are off. [Branch model](index.html#L125) · [Bridge](index.html#L178) · [Reference channel inertia](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/transistor-part.js#L293)
- The bridge, ring, and amplifier use separate branch loops around return idlers. Board tracks sketch the electrical connections. The layouts are fixed presets and do not provide a physical assembly plan. [Layout data](index.html#L156) · [Tracks](index.html#L927)
