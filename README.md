# spintronics-3d

[Spintronics](https://upperstory.com/en/spintronics/) is a kit from Upper Story (the Turing Tumble people) where chains and sprockets act like electronic circuits. Chain tension is voltage, chain speed is current. A resistor is an oil damper, a capacitor is a spiral spring, an inductor is a flywheel, and a junction is a planetary differential, which is what makes Kirchhoff's current law hold. Steve Mould has [a good video](https://www.youtube.com/watch?v=QrkiJZKJfpY) on it.

Upper Story's own [simulator](https://simulator.spintronics.com) is 2D and treats each part as a rigid body with a tuned damping or spring constant. This repo does two things it doesn't:

1. A 3D circuit bench (`sim/codex/`) that solves the ideal circuit and draws the sprockets and chain from the solution. It adds an AC crank, which isn't an official part, so you can build a bridge rectifier.
2. First-principles models of the mechanisms themselves (`sim/mech/`), each measured like a component on a bench (V–I sweep, step response, impedance vs frequency) to see where it departs from the ideal element.

Some results so far, all from simulation with estimated parameters:

- The oil-damper resistor is not ohmic. Silicone oil shear-thins, so the 50 Ω part reads about 70 Ω at 0.25 V and 43 Ω at 6 V. An eddy-current damper would be linear.
- The kit transistor is a spring-loaded gate pressing a friction brake. Stiction gives it a real threshold with hysteresis, closer to a Schmitt trigger than the smooth damping law in the official simulator.
- A spinning gyroscope is a passive gyrator, which electronics can only do with active parts. Gyroscope plus spring makes a synthetic inductor within 1.5% of the kit flywheel at low frequency, until a gimbal resonance at 3.25 Hz.
- An LC circuit made of kit parts rings within 0.3% of the ideal frequency with Q around 16. The chain's polygonal action on the sprockets adds 1–3% speed ripple.

## Run it

Everything is static HTML with three.js from a CDN. Serve the repo root and open a page:

```
python3 -m http.server 8000
# http://localhost:8000/sim/codex/            circuit bench, 7 presets, ?test=1 runs self-tests
# http://localhost:8000/sim/mech/<part>/      damper, differential, gyrator, transistor, chain
```

Each `sim/mech/<part>/` has `model.js` (the mechanism, a pure ES module with a common port API), `bench.mjs` (`node bench.mjs` characterizes it), `results.json`, and `REPORT.md` (derivation, parameter sources, departures from ideal). Constants are marked as spec, derived, or estimate.

## Notes

- `notes/how-it-works.md`: every kit part, how the real one works and how the official simulator models it, with file:line references.
- `notes/references.md` (and `.json`): Spintronics sources, mechanical–electrical analogies (Maxwell, Firestone 1933, Smith's inerter, bond graphs), mechanical computers, papers.

## How this was made

The code was written by coding agents (OpenAI Codex, plus a GLM run in `sim/glm/` that we didn't keep building on), working from the briefs in `briefs/` and reviewed by Claude. `tasks/hillclimb-plan.md` is the plan for leaving it running: a frozen evaluator, gates that can't regress, and a loop that only merges score improvements.

The official simulator's source ([upperstory/spintronics-simulator](https://github.com/upperstory/spintronics-simulator), CC BY-NC-SA 4.0) was read for its part models. None of its code or art is included here. The one thing carried over is its transistor damping law, `100^(2(1-r)) - 1`, reimplemented for comparison and cited where used.

MIT licensed, see `LICENSE`.
