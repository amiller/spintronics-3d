# Gyroscope gyrator

The two chain ports act on orthogonal gimbal pivots. The rotor supplies angular momentum, and the gimbals exchange power through the resulting cross torques. Boudon et al. represent the Newton–Euler gyroscopic term as a multiport gyrator in their bond graph. Tellegen gives the antisymmetric cross-impedance of the ideal two-port. [Boudon et al., Eq. (2) and Fig. 2](https://journals.sagepub.com/doi/10.1177/1687814019834153) · [Tellegen, US2647239A, description and Eq. (4)](https://patents.google.com/patent/US2647239A/en)

## Derivation

Use the signed SI chain quantities `I = rω` in m/s, `V = τ/r` in N and `Q = rθ` in m. Both ports use the same pitch radius, with positive effort and flow directed into the part so `VI = τω`. One spin coulomb is 10 m of chain. Preserving watts gives `I_SI = 10 I_spin`, `V_SI = 0.1 V_spin`, `C_SI = 100 C_spin`, and `L_SI = L_spin/100`. [Chain and units](../../../notes/how-it-works.md#L5) · [Upper Story science guide](https://upperstory.com/en/spintronics/science/)

The outer angle `θ₁` rotates about +x. The inner angle `θ₂` rotates about the outer frame’s +z. The rotor spins about its local +y. With rotor mass `m`, radius `a` and thickness `t`, integrating the uniform cylinder’s squared distances from the axes gives `Jspin = ma²/2` and `Jtr = m(3a²+t²)/12`. At neutral, each pivot sees `Jᵢ = Jtr + Jgᵢ`. If `ψ` is the rotor’s relative spin angle, projecting the outer pivot rate onto the spin axis gives the axial rate `ψ̇ − ω₁ sin θ₂`. Its conjugate momentum is `H = Jspin Ω`, with fixed axial rate `Ω`. Substituting the spin constraint into `T − Hψ̇` leaves the coupling term `−Hω₁ sin θ₂` alongside the transverse kinetic energy and an additive constant. Linearizing about neutral and applying the Euler–Lagrange equations with viscous pivot drag gives

```text
τ₁ = J₁ω̇₁ + b₁ω₁ − Hω₂
τ₂ = J₂ω̇₂ + b₂ω₂ + Hω₁ + kθ₂
```

Here `k = 0` for the bare two-port. Use `Lᵢ = Jᵢ/r²`, `Rᵢ = bᵢ/r²` and `G = H/r²`. The cross terms give `V₁ = −GI₂, V₂ = GI₁` in the ideal limit. They cancel in `V₁I₁ + V₂I₂`, so the stored energy `E = (J₁ω₁² + J₂ω₂² + kθ₂²)/2` changes by input power minus `b₁ω₁² + b₂ω₂²`. The bare free rates have poles at `−σ ± jωd`, where `σ = (R₁/L₁ + R₂/L₂)/2` and `ωd² = G²/(L₁L₂) − (R₁/L₁ − R₂/L₂)²/4`. This oscillatory transient is the model’s nutation. [Midpoint port equations](model.js#L68)

### Capacitive termination

Wire a torsion spring across port 2. Its current is `−I₂` and `k = r²/C`. The API’s port-2 torque is then the external effort applied to the combined shaft, so pass `{torque: 0}` to leave the spring as its only external load. Eliminating port 2 yields

```text
Zin(s) = R₁ + sL₁ + G² / (R₂ + sL₂ + 1/(sC))
       ≈ R₁ + s(L₁ + G²C)     as s → 0
```

For an ideal gyrator, `Lsynthetic = G²C` at all frequencies. In that limit the spring displacement satisfies `Q₂ = −CGI₁`. The spring energy `Q₂²/(2C)` equals `G²C I₁²/2`, the same stored energy law as a flywheel of the reflected inertance. `R₁` dominates the response at DC. Finite `L₂` and `C` create a peak around `1/(2π√(L₂C))`, broadened by `R₂`. Above the peak the spring branch contributes negative reactance until the input gimbal inertia dominates. [Bench sweep and fit](bench.mjs#L109)

### Integration and API

`create(params)` exposes `state`, `ports`, `step(dt, drive)` and derived `properties`. Pass two commands such as `[{torque: 1e-4}, {speed: 0}]` in N·m and rad/s. The interval is a positive integer multiple of `fixedStep`. Torque holds over the interval and prescribed speed ramps to the requested end rate. The port objects report interval means for both quantities and their SI chain equivalents. `state.omega` and `state.angle` are endpoint rates and angles. Invalid commands or intervals throw. [Model API](model.js#L51)

Implicit midpoint solves both pivot rates together. It advances `θᵢ` by `h ωᵢ,mid` and evaluates the spring at `θ₂,mid`. Multiplying the equations by `hωᵢ,mid` yields the exact quadratic energy increment for each tick. With zero drive it conserves energy at zero drag and dissipates it at positive drag. [Midpoint solve](model.js#L68) · [Work–energy check](bench.mjs#L133)

## Parameters

| Parameter | Default and units | Source |
| --- | --- | --- |
| `radius` | 0.020584 m | Reference simulator’s resistor sprocket, reused for both concept ports. [Notes](../../../notes/how-it-works.md#L46) |
| `rotorMass`, `rotorRadius`, `rotorThickness` | 0.006 kg, 0.032 m, 0.005 m | Estimates for a lightweight disk sized to fit inside two rings. |
| `spinRate` | 285 rad/s (2,722 rpm) | Estimate selected to bring `L₁ + G²C` near the nominal flywheel inertance. Negative values reverse gyration. |
| `gimbalInertia1`, `gimbalInertia2` | 1.1 × 10⁻⁵, 7 × 10⁻⁶ kg·m² | Estimates including non-rotor moving hardware. Equivalent to roughly 9 g at 35 mm and 6 g at 34 mm, respectively. |
| `drag1`, `drag2` | 20, 30 µN·m·s/rad | Estimates of weak pivot drag, chosen to damp free nutation over several cycles. |
| `capacitance` | 0 m/N bare, 0.12 m/N in demo | Zero disables the spring. The loaded value converts the kit’s 1.2 spin mF rating. [Upper Story capacitor](https://store.upperstory.com/collections/spintronic-parts/products/spintronic-capacitor-1-2-mf) |
| `fixedStep` | 1/3840 s | Numerical choice giving 60 steps per cycle at the sweep endpoint, 64 Hz. [Defaults](model.js#L5) |
| Comparison flywheel | 0.550 kg inertance | Converts the nominal 55 spin H rating, with ideal `V = L dI/dt`. The physical part uses steel flyweights to brake at high current. Brake and drag curves are unverified. [Upper Story inductor](https://store.upperstory.com/collections/spintronic-parts/products/spintronic-inductor-56-h) |

## Bench

Run `~/.local/bin/node sim/mech/gyrator/bench.mjs`. The signed DC sweep holds the other shaft at rest and samples at constant rate immediately after its initial ramp, across ±0.04 m/s on each axis. The bare step holds port 2 at zero torque. The AC sweep projects whole settled cycles onto sine and cosine, with each impedance column measured in turn at zero current on the other port. The loaded sweep holds the extra port-2 effort at zero. [Bench implementation](bench.mjs#L22) · [Run data](results.json)

| Quantity | Simulated fit or sample | Reference |
| --- | ---: | ---: |
| Bare `Z₁₂`, `Z₂₁` (N·s/m) | −2.06636, +2.06636 | −G, +G |
| DC `R₁`, `R₂` (N·s/m) | 0.047203, 0.070805 | 0, 0 |
| AC `L₁`, `L₂` (kg) | 0.029619, 0.020178 | 0, 0 |
| Bare step nutation and decay | 13.4467 Hz, 2.5506 s⁻¹ | 13.4529 Hz, 2.5516 s⁻¹ from the poles |
| Bare `I₂`, 2.5 s after +0.003 N | −0.0014524 m/s | −0.0014518 m/s from ideal gyration |
| Low-frequency synthetic `L` (kg) | 0.541962 | 0.512383 ideal gyrator + spring, 0.550 kit flywheel |
| Low-frequency synthetic `R`, inferred load `C` | 0.046923 N·s/m, 0.119991 m/N | 0.047203 N·s/m, 0.120000 m/N |
| Loaded `I₁`, 0.60 s after +0.018 N | 0.021051 m/s | 0.019636 m/s nominal flywheel |

The synthetic input is 1.46% below the nominal flywheel inertance in the low-frequency limit. At 0.5 Hz it measures 1.7434 N·s/m at +86.97°, versus 1.7279 N·s/m at +90° for the flywheel. At 3.25 Hz it peaks on the grid at 60.23 N·s/m and −2.58°. The undamped denominator predicts 3.235 Hz. At 8 Hz the loaded phase is −82.46°. The DC sweep’s largest linear residual is \(6.73\times10^{-17}\) N, consistent with zero modeled threshold. The largest bench tilt is 0.2325 rad and the work–energy residual is \(7.90\times10^{-23}\) J per tick. All final bench assertions passed. The initial constant-inductance fit through 1 Hz failed because the curve had already risen. The final fit extrapolates from 0.125–0.5 Hz against squared frequency. The initial bare step also exceeded the linearization window, so its drive was reduced to 0.003 N. [Fit and checks](bench.mjs#L175) · [Full frequency sweep](results.json)

The bare two-port assumes a balanced rotor, fixed axial spin, aligned no-slip shafts and small inner tilt. At finite tilt the cross coefficient contains `cos θ₂` and the projected transverse inertia varies. At 0.25 rad the change in the cosine factor is 3.11%. Constant current can accumulate gimbal angle, so the DC sweep checks a local law. Spin regulation, travel stops, breakaway friction, chain elasticity and fixed-board routing for the moving inner axis are unverified. The cutaway routes that chain on the outer frame and plays rotor spin at 1/24 speed. [Gimbal and chain geometry](index.html#L228) · [Model animation](index.html#L496)

## Fit into the ideal-circuit solver

Replace one ideal `l` branch with the spring-terminated mechanism for a one-port drop-in. A bare gyrator instead needs two oriented branches, each with its own terminal pair. Keep the network in one unit system. If the network uses numeric spin units, convert `I` and `V` at the adapter boundary using the factors above. For a target low-frequency `L` in SI, set `|G| = √((L − L₁)/C)`, which requires `L ≥ L₁`, and set `Ω = G r²/Jspin`. [Reference branch and companion](../../codex/index.html#L299)

Use `create({fixedStep: 1/480, capacitance: C})` for the reference solver’s single tick per accepted network step. Let `D = diag(L₁,L₂)`, `B = [[R₁,−G],[G,R₂]]` and `S = diag(0,1/C)` for the built-in spring or zero for the bare gyrator. Given endpoint history `Iₙ = rωₙ`, `Qₙ = rθₙ`, the model’s midpoint law is

```text
V̄ = M Iₙ₊₁ + b
M = D/h + B/2 + hS/4
b = (−D/h + B/2 + hS/4) Iₙ + S Qₙ
```

Stamp `Iₙ₊₁ = M⁻¹(V̄ − b)` into KCL with both branch incidence vectors for the bare part. For the one-port set `V̄₂ = 0` and eliminate row 2, giving `M₁₁ − M₁₂ M₂₁/M₂₂` and `b₁ − M₁₂ b₂/M₂₂`. In the existing endpoint solve take `V̄ = (1−θ)Vₙ + θVₙ₊₁` using the solver’s integration weight and including its drag effort. Convert accepted `V̄` to torques via `r V̄` and call `step` once after candidate acceptance. Use endpoint `rω` for branch current and `rθ` for chain travel. The model’s interval mean is useful for the scope and power checks. [Reference step size](../../codex/index.html#L127) · [KCL assembly and candidate acceptance](../../codex/index.html#L325) · [Chain animation conversion](../../codex/index.html#L892)

The companion and Schur reduction matched the model’s endpoint currents within \(1.04\times10^{-17}\) m/s in the bench. [Companion check](bench.mjs#L151) · [Run data](results.json)

## View and verification

Serve this directory over HTTP and open [index.html](index.html). Use **Cross-port** with the other axis **Held at rest** to see the signed cross-impedance. Use **Synthetic inductor** to overlay the ideal nominal flywheel in blue under the same voltage or current drive. The smoke test exercised the scene graph, both circuit and source modes, spin reversal, pause, reset and resize in Node with local Three.js r177 and a mock renderer. Browser/WebGL rendering with the pinned CDN r160 import is unverified. The sandbox blocked Chromium startup, Firefox exited during startup, and CDN DNS resolution failed. [View imports and controls](index.html#L8)
