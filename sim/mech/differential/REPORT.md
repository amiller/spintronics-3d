# Planetary junction

## Geometry and port convention

Upper Story identifies the bottom sprocket with the carrier, the middle with the ring, and the top with the sun. Its specified rim-speed law is $v_b=v_m+v_t$. [Upper Story junction, detailed description](https://store.upperstory.com/products/spintronic-junction-value-pack)

Use mechanical SI in the impedance analogy. For each shaft, $x=r\theta$, $I=r\omega$ in m/s and $V=\tau/r$ in N, so $VI=\tau\omega$ in W. Positive torque acts on the part. Compare through-efforts as $V_b=V_{\rm bottom}$, $V_m=-V_{\rm middle}$ and $V_t=-V_{\rm top}$ with bottom as input. All current coordinates use the same shaft rotation sign. [Chain and unit derivation](../../../notes/how-it-works.md#L5)

| Geometry | Value and basis |
| --- | --- |
| Chain pitch radii $r_b,r_m,r_t$ | 33.941, 22.6273, 11.3137 mm. Anchor the documented simulator bottom radius and set 3:2:1 for exact rim-speed KCL. The simulator uses 33.941, 22.638, 11.348 mm with a 3:2:1 weighted speed law. [Notes and source](../../../notes/how-it-works.md#L72) |
| Planet arrangement | Three planets of pitch radius $R_p=3$ mm on orbit $a=9$ mm. Values are from the simulator's inactive sketch. The sketch also gives the 48:12 ring-to-planet tooth ratio. Physical dimensions and counts are **unverified**. [Simulator geometry](../../../ref/spintronics-simulator/public/parts/junction-part.js#L4) · [Planet sketch](../../../ref/spintronics-simulator/public/parts/junction-part.js#L132) · [Tooth ratio](../../../ref/spintronics-simulator/public/parts/junction-part.js#L155) |
| Sun and ring | $R_s=a-R_p=6$ mm and $R_r=a+R_p=12$ mm. Equal pitch per tooth gives 24:48:12 for sun, ring, planet. |

## Equations

Write $q=(\theta_b,\theta_m,\theta_t,\theta_p)^T$ in absolute angles. Identical, equally loaded planets start in matching mesh phases, so one spin coordinate represents the three. Matching pitch-line motion in the carrier frame gives the signed mesh displacements.

```text
g_s = R_s (θ_t − θ_b) + R_p (θ_p − θ_b)
g_r = R_r (θ_m − θ_b) − R_p (θ_p − θ_b)
G = ∂g/∂q
```

The plus sign is external meshing and the minus sign is internal meshing. In rigid engagement, $\dot g=0$, so $(R_s+R_r)\omega_b=R_s\omega_t+R_r\omega_m$. With $\kappa=R_s/r_t=R_r/r_m$, $g_s+g_r=\kappa(x_t+x_m-x_b)$. Thus $I_b-I_m-I_t=-(\dot g_s+\dot g_r)/\kappa$. The rigid limit gives $I_b=I_m+I_t$. With zero inertia and losses, $V_b I_b-V_m I_m-V_t I_t=0$. Independence of $I_m,I_t$ then gives $V_b=V_m=V_t$.

Planet orbit speed is $a\omega_b$. Adding translational and rotational kinetic energies gives $T=\tfrac12\omega^T D\omega$, where $D=\operatorname{diag}(J_b+n m_p a^2,J_m,J_t,nJ_p)$ and $n=3$. A tangential force $f_j$ per planet contributes generalized torque $-nG_j^T f_j$ by virtual work.

Each mesh flank has penetration $p_\sigma(g)=[\sigma g-b]_+$, where $\sigma=\pm1$, $b$ is half-clearance and $[y]_+=\max(0,y)$. The step force per flank is a spring and dashpot projected onto compression.

```text
f⁺ = Σσ σ [k p_σ(g⁺) + c (p_σ(g⁺) − p_σ(g))/h]₊
D(ω⁺ − ω)/h = τ⁺ − B ω⁺ − n Gᵀ f⁺
g⁺ = g + h G ω⁺
```

$B$ includes bearing drag and tooth sliding. At the pitch point the rigid gear surfaces roll together. Off that point, a displacement of the contact point contributes a relative-velocity term proportional to $\omega_{\rm mate}-\omega_p$. The model approximates sliding drag with paired, opposing torques and a coefficient per planet $d_j=c_f\min(1,\lvert f_j\rvert/F_0)$ held from the previous step. Sliding dissipates $n\sum_j d_j(\omega_{\rm mate}-\omega_p)^2$.

Backward Euler eliminates prescribed speeds, solves the two-mesh contact problem over feasible flank states, then recovers speed-source torques from the momentum equation. A flank that unloads fast enough goes force-free. Its discrete work is at least its change of spring energy $k p^2/2$. The inertia term adds the nonnegative backward-Euler loss $\|\omega^+-\omega\|_D^2/2$, and $B$ is positive semidefinite. This gives the discrete energy inequality checked by the bench. The model assumes rigid shafts, identical planet contacts, linear flank stiffness, and viscous losses. It omits Coulomb stiction, tooth-to-tooth variation, and chain dynamics. [Force and integration implementation](model.js#L187)

## Default parameters

The inertia estimates use uniform disks with $J=mr^2/2$ and a ring annulus with $J=m(r_i^2+r_o^2)/2$, from integrating $r^2\,dm$. Chosen masses correspond to a few millimetres of effective section thickness at an assumed density of 1,000 kg/m³. Physical masses, contact constants and losses are **unverified**. [Executable defaults](model.js#L27)

| Parameter | Default | Basis |
| --- | ---: | --- |
| `carrierInertia` | 5.760 × 10⁻⁶ kg m² | Estimate. 10 g disk at $r_b$ including the bottom sprocket. Planet orbit inertia is added separately. |
| `ringInertia` | 2.624 × 10⁻⁶ kg m² | Estimate. 8 g annulus from $R_r$ to $r_m$ including the middle sprocket. |
| `sunInertia` | 1.920 × 10⁻⁷ kg m² | Estimate. 3 g disk at $r_t$ including the top sprocket. |
| `planetInertia`, `planetMass` | 5.40 × 10⁻¹⁰ kg m², 0.12 g per planet | Estimate. One disk at $R_p$. |
| `bottomDrag`, `middleDrag`, `topDrag` | 3, 2, 1 × 10⁻⁶ N m s/rad | Estimate. Small bearing loss, increasing with shaft assembly size. At 10 rad/s these imply 10–30 µN m. |
| `meshStiffness` | 20,000 N/m per planet per mesh | Estimate. 1 N tangential load per 50 µm deflection. |
| `meshDamping` | 0.6 N s/m per planet per mesh | Estimate. Subcritical tooth damping using one planet's pitch-line mass $J_p/R_p^2$ and $2\sqrt{kJ_p/R_p^2}\simeq2.19$ N s/m as a scale. |
| `backlash` | 50 µm half-clearance per mesh | Estimate. Small fraction of the 1.57 mm tooth pitch implied by the sketch. |
| `meshFriction`, `frictionLoad` | 1.5 × 10⁻⁶ N m s/rad, 0.1 N per planet per mesh | Estimates. Use small sliding drag (15 µN m at 10 rad/s relative spin) at a reference load of order one planet's share of the blocked bench load. |
| `fixedStep` | 1/9600 s | Numerical choice. Twenty mechanism steps per reference 1/480 s tick. [Circuit bench tick](../../codex/index.html#L127) |

## Bench

Run from the repository root.

```sh
~/.local/bin/node --experimental-default-type=module sim/mech/differential/bench.mjs
```

The DC sweep fixes compatible speeds in each branch from −0.24 to +0.24 m/s with the other branch locked and both meshes preloaded to 0.25 N per planet. In the top-only +0.24 m/s case, bearing and sliding losses are 0.600 and 10.125 mW. The AC sweep applies 0.003 m/s sinusoidal excitation to one output current at a time about a 0.04 m/s bias on each. It measures the two conjugate drops $(V_b-V_m,V_b-V_t)$ by synchronous demodulation at 0.5–192 Hz. The fit uses 0.5–8 Hz with $Z\approx R+j2\pi fL$. [Bench procedure](bench.mjs) · [Full DC, AC and transient samples](results.json)

| Metric | Model | Ideal junction |
| --- | ---: | ---: |
| DC $R_{mm},R_{mt}=R_{tm},R_{tt}$ (N s/m) | 0.050456, −0.085287, 0.186199 | 0 |
| Fitted $L_{mm},L_{mt}\simeq L_{tm},L_{tt}$ (kg) | 0.010163, 0.0050133, 0.0065366 | 0 |
| Blocked compliance (µm/N) | 118.518 | 0 |
| Blocked reversal dead band (µm) | 377.29 | 0 |
| Blocked +0.15 N step peak $\lvert I_b\rvert$ (m/s) | 0.10473 | 0 |
| Both meshes engaged after that step (ms) | 3.542 | 0 |
| Mixed-drive steady KCL residual (m/s) | 2.78 × 10⁻¹⁷ | 0 |

At 1 Hz, diagonal impedances are $0.08140\angle51.67^\circ$ and $0.19069\angle12.44^\circ$ N s/m for middle and top. At 32 Hz they are $2.0444\angle87.99^\circ$ and $1.3293\angle81.35^\circ$. The rigid prediction uses $\omega=A[I_m,I_t]^T$, with rows $[1,1]/r_b$, $[1,0]/r_m$, $[0,1]/r_t$, and $[1,-1]/r_t$. Then $L=A^TDA$ and $R=A^TBA$. Coupling comes from shared inertias and sliding coordinates. The negative off-diagonal loss term comes from opposing relative speeds, and $I^T R I\ge0$.

With both outputs locked, each mesh carries $\lvert V_b\rvert/(n\kappa)$ in static balance. Two mesh springs give $C=2/(nk\kappa^2)$ and traversing both backlash gaps predicts $4b/\kappa=377.12$ µm. The +0.15 N step ends at 206.34 µm of bottom travel. Reversing to −0.15 N seats both opposite flanks after 4.792 ms. The gap is a travel threshold. During contact transitions $I_b-I_m-I_t$ can be nonzero, and no breakaway tension is modeled.

The model assertions passed, including discrete energy and mixed-drive KCL. Refining from $h$ to $h/16$ changed the positive-step peak from 0.10473 to 0.10752 m/s and engagement from 3.542 to 3.555 ms. The 2% settling time changed from 36.8 to 134 ms. Backward-Euler damping makes ring-down duration timestep-sensitive. These are results for the stated estimates. Physical calibration is **unverified**. [Recorded results and refinement](results.json)

## API and reference simulator

```js
import { create, defaults } from './model.js';
const junction = create();
junction.step(defaults.fixedStep, {
  bottom: {speed: 3},       // rad/s
  middle: {speed: 0},
  top: {torque: -0.002},    // N m
});
const {current, voltage} = junction.ports.bottom;
```

Omitted ports receive zero torque. Each port reports speed (rad/s), torque (N m), current (m/s), voltage (N), radius and travel (m). The live `state` exposes time (s), shaft angles (rad), angular speeds (rad/s), mesh displacement (m), signed tangential force (N per planet), flank (−1, 0, +1), stored energy (J), bearing and sliding loss (W), and current residual (m/s). `step` holds commands over an integer number of fixed steps and reports the final substep. A speed discontinuity includes the reaction impulse distributed over its first substep. [Port implementation](model.js#L148)

The reference parallel preset collapses the junction to node `a`. Split that node into bottom, middle and top terminals for `A1`, `R1` and `R2`. Use a three-port companion from those terminals to the common reference. For a synchronized solve, advance MNA at the mechanism timestep and put the two flank modes into the existing candidate/feasibility loop. [Parallel preset](../../codex/index.html#L209) · [MNA stamping](../../codex/index.html#L325) · [Accept and advance](../../codex/index.html#L363)

For each candidate, freeze the previous sliding weights and write $H\omega^+=s+Pu$, with $P$ the signed radius columns $[r_b e_b,-r_m e_m,-r_t e_t]$ and $u$ the terminal efforts in N. For active rows $G_A$, let $K=k+c/h$ and let $d_A$ be the signed contact thresholds from the previous penetrations. Then $H=D/h+B+nhK G_A^T G_A$ and $s=D\omega/h-nK G_A^T(g_A-d_A)$. Stamp $i=P^T H^{-1}P u+P^T H^{-1}s$ and verify the candidate flank conditions before committing. To interpret MNA values as kit spin units, use $u=V_{\rm spin}/10$ and $I_{\rm spin}=i/10$. This divides the stamped conductance by 100 and its history current by 10. Commit once via `step(h, drive)` with torques $Pu$. Use model angles for the three sprockets and the carrier, and `planet − bottom` for each planet parented to the carrier. Drive chain phase from signed port travel. [Kit units](https://upperstory.com/en/spintronics/science) · [Mesh thresholds](model.js#L202) · [Reference gear and chain rendering](../../codex/index.html#L694) · [Chain phase](../../codex/index.html#L883)

Serve this directory over HTTP and open `index.html`. The cutaway enlarges the mesh relative to the sprocket stack and uses schematic tooth profiles. The scene, scope and controls passed a Node smoke test with local three.js r177 and a stub DOM/renderer. Browser rendering on pinned r160 is **unverified**. The sandbox blocked the HTTP server, Chromium startup and jsDelivr DNS resolution. [Demo](index.html)
