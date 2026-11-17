# Spintronics transistor

## Hardware evidence and units

Upper Story describes three arms squeezing an O-ring on the bottom path sprocket. Turning the top gate releases the arms. Its first injection-molded sample measured about 6 kΩ at zero gate and 2 spin mA, with a lower apparent resistance at higher path current. Mould demonstrates the clamps holding the ring and releasing under gate load at 10:07–10:23. Three brake pieces and three moving balls appear in the simulator art. The exact production cam profiles, ball contacts, O-ring dimensions, and breakaway force are **unverified**. The original cutaway shows an inferred linkage and exaggerates its movement. The campaign photo and animation links returned HTTP 410 during inspection. The reference base drawing shows a threefold layout. [Upper Story sample tests](https://spintronics.backerkit.com/hosted_preorders/project_updates?page=3) · [Mould transcript](https://lilys.ai/ko/notes/638148) · [Video at 10:07](https://www.youtube.com/watch?v=QrkiJZKJfpY&t=607s) · [Simulator brake and balls](../../../ref/spintronics-simulator/public/parts/transistor-part.js#L43) · [Simulator ball movement](../../../ref/spintronics-simulator/public/parts/transistor-part.js#L262) · [Base drawing](../../../ref/spintronics-simulator/public/Images/Transistor%20Base.pdf)

Take $Q=r\theta$ in m, $I=r\omega$ in m/s and $V=\tau/r$ in N. Then $VI=\tau\omega$ in W. The kit’s 10 m per spin C gives $I_{\rm spin}=I/10$, $V_{\rm spin}=10V$ and $Q_{\rm spin}=Q/10$. Use $r_g=31.886$ mm and $r_d=16.990$ mm from the reference simulator. The catalog’s 2.2 millispin C gives 22 mm of gate travel, or 0.690 rad above the closed stop at this radius. The official simulator instead traverses 38.52° between its stops. The model takes the catalog’s approximately 1.5 spin V as its full-open gate effort, 0.15 N. [Upper Story science guide](https://upperstory.com/en/spintronics/science/#spintronic-and-electronic-units) · [Upper Story catalog](https://store.upperstory.com/products/spintronic-transistor) · [Mechanism notes](../../../notes/how-it-works.md#L89) · [Simulator radii and stops](../../../ref/spintronics-simulator/public/parts/transistor-part.js#L4)

## Equations

For the spring gate, $q=s\theta_g\in[0,q_m]$ with setup polarity $s=\pm1$. Assume a constant cam lead $a=\delta_0/q_m$ and lump the three arms and ring into normal stiffness $k_p$. Compression $\delta=\delta_0-aq$ gives total normal force $N=k_p\delta$. Its stored energy $k_p\delta^2/2$ contributes $-aN$ to resisting gate torque. A preloaded torsion spring with resistance $a k_p\delta_0+k_t q$ cancels that term at closure. The net slope is $k_g=k_t+k_p a^2=r_gV_{\rm full}/q_m$. Thus inside the stops,

$$
J_g\ddot q + b_g\dot q + k_gq = s\tau_g.
$$

Stop reactions bound the travel. A speed source determines the gate torque by inverse dynamics. With $Q=r_g\theta_g$, the linear gate equivalent is a series combination $C_g=r_g^2/k_g$, $R_g=b_g/r_g^2$ and $L_g=J_g/r_g^2$. [Gate implementation](model.js#L67)

The arms react against the frame and act at drum radius $r_b$. With channel angle $\theta_d$ and speed $\omega_d$,

$$
J_d\dot\omega_d=\tau_d-b_d\omega_d-\tau_f,\qquad
\tau_f=
\begin{cases}
\mu_k N r_b\,\mathrm{sgn}(\omega_d), & \omega_d\ne0,\\
\tau_d, & \omega_d=0,\ |\tau_d|\le\mu_s N r_b.
\end{cases}
$$

A drive past the static limit launches slip using the kinetic coefficient. At a frozen gate, sliding gives $V_{DS}=L_d\dot I_D+R_d I_D+\mathrm{sgn}(I_D)F_k$, with $L_d=J_d/r_d^2$, $R_d=b_d/r_d^2$ and $F_k=\mu_k N r_b/r_d$. From rest, $I_D=0$ for $|V_{DS}|\le F_s=\mu_s N r_b/r_d$. Above breakaway the slope is $1/R_d$. During unloading, existing slip can persist to $F_k$. For interior gate travel on a settled sliding branch, $g_m=\mathrm{sgn}(I_D)F_{k,\mathrm{closed}}/(R_d V_{\mathrm{full}})$. The stuck branch has zero local $g_m$. [Channel implementation](model.js#L116)

### Flyball alternative

Two equal balls slide on opposite radial guides on the gate shaft. Let $M$ be their total mass, $x\in[0,x_m]$ their outward stroke, and $\rho$ the ball-center radius at rest. An ideal linkage of displacement ratio $\lambda=\delta_0/x_m$ presses the brake by $\lambda x$. Its reflected spring stiffness is $k_p\lambda^2$ by differentiating the pad energy $k_p(\lambda x)^2/2$ with respect to $x$. The rotor’s angular momentum is $J(x)\omega_g$ with $J(x)=J_h+M(\rho+x)^2$. Differentiating the kinetic energy $M\dot x^2/2+J(x)\omega_g^2/2$ gives the centrifugal term $M(\rho+x)\omega_g^2$. Thus

$$
\begin{aligned}
\frac{d}{dt}[J(x)\omega_g]&=\tau_g-b_{\rm gov}\omega_g,\\
M\ddot x + c_x\dot x+F_0+(k_x+k_p\lambda^2)x&=M(\rho+x)\omega_g^2,\\
N&=k_p\lambda x.
\end{aligned}
$$

At constant speed, solve the radial balance and apply its two stroke limits. In the interior, $x=[M\rho\omega_g^2-F_0]/[k_x+k_p\lambda^2-M\omega_g^2]$. In positive channel slip, $\beta=\partial I_D/\partial I_B=-\mu_k r_b k_p\lambda\,(\partial x/\partial I_B)/(r_d R_d)$, where $I_B=r_g\omega_g$. Direct engagement reduces positive $I_D$ as positive $I_B$ rises. Gain depends on bias, falls to zero on the steady travel plateaus, and is discontinuous at path breakaway. The gate absorbs shaft power $b_{\rm gov}\omega_g^2$ at steady speed. The drive on the path supplies its power. [Governor implementation](model.js#L91) · [Beta sweep](bench.mjs#L142)

### Time stepping

One public step is $h=1/480$ s. Backward Euler handles the spring, drag and sliding inertia. Travel stops project the candidate displacement onto its interval and use the resulting step-average speed on a hit. The channel first tests whether the kinetic branch preserves the previous direction. If it reaches zero, the static inequality decides whether it stays there. For a reversal the available launch time is $h-J_d|\omega_{d,\rm old}|/(\mu_k N r_b-\tau_d\mathrm{sgn}(\omega_{d,\rm old}))$. The reported channel angle uses endpoint speed. The governor solves radial return stiffness and damping implicitly, evaluating centrifugal force at the old radius with the driven angular speed. Under torque drive it transports angular momentum through the radial displacement after applying drive and drag at the old radius. This split is first order. Brake clamp force uses the new actuator position and is held constant over the channel update. [Fixed-step implementation](model.js#L67) · [Step validation and commit](model.js#L179)

## Parameters

| Parameters | Defaults and provenance |
| --- | --- |
| `mechanism`, `gateDirection` | `spring` and +1 by convention. `governor` selects the alternative. Direction is dimensionless, and the kit lets the user choose its operating polarity. [Upper Story catalog](https://store.upperstory.com/products/spintronic-transistor) |
| `gateRadius`, `channelRadius`, `gateTravel`, `fullGateEffort` | 0.031886 m, 0.016990 m, 0.690 rad, and 0.15 N. Simulator radii and catalog conversions above. |
| `gateInertia`, `channelInertia`, `governorHubInertia` | Estimates of $7\times10^{-6}$, $2\times10^{-6}$, and $4\times10^{-6}$ kg·m². Equivalent uniform disks of about 14 g, 14 g and 8 g at the corresponding pitch radii using $J=mr^2/2$. |
| `gateDrag`, `channelDrag`, `governorDrag` | Estimates of $2.5\times10^{-4}$, $2.0\times10^{-4}$ and $1.2\times10^{-4}$ N·m·s/rad. Chosen for a damped gate return, a small on-state slope, and light running loss at the governor input. |
| `drumRadius`, `padTravel`, `padStiffness` | Estimates of 0.012 m, 0.0006 m, and 5500 N/m total. The ring sits within the channel pitch circle in the model. The sub-mm compliance and estimated kinetic coefficient give 1.179 N sliding effort at 0.020 m/s, close to the first-sample 1.2 N inferred from 6 kΩ × 2 spin mA. The individual contact parameters are underdetermined by that test. [Upper Story sample tests](https://spintronics.backerkit.com/hosted_preorders/project_updates?page=3) |
| `muStatic`, `muKinetic` | Estimates of 0.65 and 0.50, dimensionless, chosen to distinguish breakaway and running friction. Production friction coefficients are **unverified**. |
| `flyballMass`, `flyballRadius`, `flyballStroke` | Design estimates of 0.010 kg per ball, 0.020 m, and 0.003 m. A pair of compact balls fits within the modeled gate pitch circle with room for outward travel. |
| `governorPreload`, `governorStiffness`, `governorDamping` | Design estimates of 0.035 N, 28 N/m, and 0.9 N·s/m. Chosen for an initial engagement near 0.3 m/s, a finite transition, and damped radial motion. |

## Bench results

Run `~/.local/bin/node --experimental-default-type=module sim/mech/transistor/bench.mjs` from the repository root. The no-flag invocation also runs under the installed Node 22.11.0 with a module-type warning. The bench records from-rest signed DC sweeps, ascending and descending ramps with 0.04 N increments, speed-driven sliding fits, step traces, and fundamental phasors at nine frequencies from 0.5 to 48 Hz. For the path phasors the gate is half open and $I_D=0.24\pm0.01$ m/s. The gate impedance uses an interior bias and 0.4 mm nominal chain-travel amplitude. The fits use $D=(1-e^{-j2\pi f h})/h$ for the derivative and $1/D$ for the integral. All values below are simulated. [Bench](bench.mjs#L76) · [Phasor extraction](bench.mjs#L168) · [Results](results.json)

For the official comparison, use opening $u=q/q_m$, $d=\max(0.5,100^{2(1-u)}-1)$, $J_P=\pi(d/100)r_d^4/2$ and $h_P=1/30$ s. With the chain prescribing speed, Planck damps before the gear constraint. The required impulse is $J_P[\omega_n-\omega_{n-1}/(1+h_Pd)]$, giving $R_P=J_P d/[r_d^2(1+h_Pd)]$ and $L_P=R_P/d$. The bench marks linear DC predictions beyond the Planck rotor speed cap and omits the official phasor at frequencies at or above 15 Hz. Both the official path calculation and the Codex ideal law $R_{\rm ideal}\propto d$ are normalized to the model’s closed-gate sliding ratio at 0.020 m/s, 58.962 N·s/m. The official SI mass/torque calibration is **unverified**. The ideal gate is a pure C with the modeled compliance. [Official update](../../../ref/spintronics-simulator/public/parts/transistor-part.js#L267) · [Planck damping](../../../ref/spintronics-simulator/public/planck.js#L3687) · [Planck rotation cap](../../../ref/spintronics-simulator/public/planck.js#L3752) · [Gear impulse](../../../ref/spintronics-simulator/public/planck.js#L10192) · [Codex resistance law](../../codex/index.html#L286) · [Bench normalization](bench.mjs#L43)

| Quantity | Spring brake | Codex ideal | Official law, normalized |
| --- | ---: | ---: | ---: |
| Gate series C (m/N) | 0.146667 | 0.146667 | — |
| Gate series R (N·s/m) | 0.245889 | 0 | — |
| Gate series L (kg) | 0.006885 | 0 | — |
| Half-open path incremental R (N·s/m) | 0.692856 | 0.583786 | 0.44937 |
| Half-open path L (kg) | 0.006929 | 0 | 0.004539 |
| Closed-gate breakaway (N) | 1.51501 | 0 | 0 |
| Closed-gate kinetic offset (N) | 1.16539 | 0 | 0 |
| Sliding $g_m$ at $V_g=0.05625$ N, $V_{DS}=1.10$ N ((m/s)/N) | 11.2134 | 36.4525 | 43.2618 |

| Gate effort (N) | Breakaway (N) | Kinetic offset (N) | From-rest $I_D$ at $V_{DS}=1.20$ N (m/s) | Official $d$ (1/s) |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 1.51501 | 1.16539 | 0 | 9999 |
| 0.0375 | 1.13626 | 0.87404 | 0.47045 | 999 |
| 0.075 | 0.75750 | 0.58270 | 0.89096 | 99 |
| 0.1125 | 0.37875 | 0.29135 | 1.31146 | 9 |
| 0.15 | 0 | 0 | 1.73196 | 0.5 |

At the same $V_g$ and $V_{DS}=0.40$ N the brake is stuck and $g_m=0$. The positive and negative DC curves mirror each other. The gate changes the Coulomb offset and sticking interval. The official and ideal paths instead retain a linear current slope at each gate travel and carry nonzero current under arbitrarily small nonzero effort. The official path also changes inertia with gate travel. The friction model retains fixed path inertia and predicts decreasing apparent $V/I=R_d+F_k/|I_D|$ with slip speed, consistent with the direction of the sample trend. The actual sample’s detailed speed and gate curves are **unverified** beyond the manufacturer’s published observations. The model omits viscoelastic lag, wear, contact-induced ripple, and chain compliance. [DC sweeps](bench.mjs#L76) · [Official density update](../../../ref/spintronics-simulator/public/parts/transistor-part.js#L289) · [Upper Story sample tests](https://spintronics.backerkit.com/hosted_preorders/project_updates?page=3)

| Frequency (Hz) | Spring path impedance (N·s/m, °) | Spring gate impedance (N·s/m, °) | Governor gate impedance (N·s/m, °) |
| ---: | ---: | ---: | ---: |
| 0.5 | 0.6933, +1.80 | 2.1635, −83.28 | 0.1266, +20.92 |
| 4 | 0.7188, +14.01 | 0.2756, −20.89 | 0.3867, +69.93 |
| 8 | 0.7915, +26.05 | 0.3429, +37.75 | 0.7588, +75.78 |
| 30 | 1.5858, +53.38 | 1.3285, +67.68 | 2.2915, +73.20 |

At 8 Hz the ideal and official path impedances are 0.5838 at 0° and 0.6149 N·s/m at +12.72°, respectively. At half gate the 0→0.95 N path step reaches 63.2% of its final 0.5301 m/s in 12.5 ms. At fixed 0.85 N path effort the 0→0.1125 N gate step first moves the channel at 50.0 ms and reaches 63.2% of its final 0.8063 m/s at 75.0 ms. The ideal path at fixed opening responds within the source step. [Step traces](bench.mjs#L117) · [Frequency results](results.json)

The flyballs begin moving at 0.2983 m/s gate chain speed and reach full engagement at 1.3122 m/s by radial force balance. At $I_B=0.80$ m/s and $V_{DS}=1.20$ N, the bench finds differential $\beta=-1.5682$, $I_D/I_B=1.5195$ and gate DC resistance 0.1180 N·s/m. The 0.5 Hz gain is 1.5684 at +177.49°. A 0→1.20 m/s gate step at 0.80 N path effort stops a previously sliding channel at 27.1 ms. Negative incremental gain is the consequence of directly engaging this brake. The ratio $I_D/I_B$ includes the path’s bias flow. [Governor tests](bench.mjs#L137) · [Frequency results](results.json)

## Integration into the Codex bench

Instantiate one model per paired `gate` and `channel` branch, keyed by the channel’s existing `gate` ID. Retain the two branch node pairs. At `Circuit.step`, use $h=\texttt{STEP}$ and backward Euler for the mechanism branches. If the circuit takes numeric values in spin units, map branch effort `v + push` to $V=0.1(v+\mathrm{push})$, then supply $\tau=rV$ to each port. Return the port current as $I/10$ in spin A. Rescale consistently for any different circuit units. Existing preset values are ideal-circuit choices. The kit defaults can shift their bias points. [Codex branch types and coupling](../../codex/index.html#L286) · [Codex stamping and branch effort](../../codex/index.html#L299)

For the simultaneous solve, free gate and sliding path stamps in SI have the form $V=ZI+V_0$. Use $Z_g=L_g/h+R_g+h/C_g$, $V_{0g}=Q_{g,\rm old}/C_g-(L_g/h)I_{g,\rm old}$, $Z_d=L_d/h+R_d$, and $V_{0d}=\mathrm{sgn}(I_D)F_k-(L_d/h)I_{D,\rm old}$. Stamp $G=1/Z$, $j=-V_0/Z$, with the drive contribution as in `passive`. At gate travel limits impose $I_g=(Q_{\rm limit}-Q_{\rm old})/h$. At a stuck path impose $I_D=0$. Evaluate clamp force from the trial gate position and check the static and moving inequalities. The signed Coulomb offset within a sliding mode is affine in gate travel and can be included as a cross-branch stamp. Use the two torque-driven currents from `preview(h, drive)` as the exact residual for mode checks and for the within-step reversal case. Commit `step(h, drive)` once with accepted torques and use each state angle or $r\theta$ to render the sprocket or chain. A blocked speed-drive command throws and leaves history intact. [Preview and step API](model.js#L154) · [Codex candidate and commit](../../codex/index.html#L325)

For the standalone cutaway, serve this directory over HTTP and open `index.html`. The bench assertions pass. A smoke test with a stub renderer and local Three.js 0.177.0 passed scene transforms, both mechanism and drive modes, sliders, the trace, pause, and reset. Rendering with the page’s pinned Three.js 0.160.0 is **unverified**. Chromium and Firefox failed to launch in the sandbox and jsDelivr DNS resolution failed. [Import map and cutaway](index.html#L7) · [Bench assertions](bench.mjs#L251)
