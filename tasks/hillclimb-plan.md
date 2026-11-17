# Leaving spintronics open: goals + hill-climbing

## Layout
```
GOALS.md            human-owned. tracks, gates, metrics, stretch goals
eval/               frozen harness. workers may not edit (diff touching eval/ = auto-reject)
  run.mjs           scores every track -> scoreboard.json, appends history.jsonl
  probes/           our own benches per element type (workers' bench.mjs is not trusted for grading)
  heldout/          tests workers never see (different params, circuits)
  page.mjs          headless chrome: loads, 0 console errors, fps, screenshot
  refs.mjs          URLs resolve, arXiv titles match, citation coverage of claims
loop/
  pick.mjs          choose target = largest weighted gap on scoreboard
  brief.mjs         turn failing metrics into a codex brief
  run.sh            git worktree -> codex exec -> eval -> merge if score up and no gate regresses
  ledger.jsonl      every attempt: target, brief, score before/after, merged?, tokens
backlog.md          agent-proposed ideas; promotion to a track needs human ok
```

## Gates (must hold, any regression = reject)
- Ideal limit: with parasitics -> 0, each mechanism matches its ideal element within 1% (R, KCL, gyrator G, C, L).
- Convergence: every reported metric changes < 2% when dt is halved twice (differential's settling time moved 37 -> 134 ms at dt/16 yet its bench said PASS).
- Energy: passive parts never create energy over a closed cycle (integral of V*I >= 0 within tolerance).
- Integration: sim/codex 7 presets still pass with every mechanical part swapped in.
- Page: loads from file:// and http, 0 console errors, >= 30 fps on fractal headless.
- Honesty: every numeric constant in model.js tagged spec|derived|estimate with source; refs.mjs passes.

## Scored metrics (climb these)
- Fidelity vs real kit data: Upper Story specs (resistor +-10% at ref voltage, 1.2 mF capacitor, +-9 V gauge, breaker), behaviour of the 4 official examples run in the official simulator locally, and (if available) measurements of a physical kit.
- Parameter provenance: fraction spec/derived vs estimate.
- Held-out circuit accuracy: mechanical-part circuits vs ideal predictions where they should agree; documented departures where they should not.
- Perf: sim steps/sec at fixed accuracy.
- Visual: LLM-judge rubric on screenshots (internals visible, motion matches model, labels), scored by a different model than the author.

## Loop policy
- Runs on fractal (cron or one long driver), N parallel worktrees, daily token budget from codex usage endpoint.
- Merge rule: weighted score strictly up, no gate down, adversarial reviewer agent finds no gaming.
- Stop a track when gates + target scores met; then only stretch goals and backlog.
- Human checkpoint: published scoreboard page (scores over time, screenshots, ledger); you approve backlog promotions and new tracks.
