# Spintronics rotary dampers

Upper Story identifies a sealed silicone-oil shear chamber, different oil blends for different labels, and ±10% resistance accuracy at a reference spin voltage. The numeric reference voltage, chamber geometry, rheology, and hardware V–I curves are **unverified**. This model uses an estimated two-face shear geometry and calibrates it at an assumed 3 spin V. The eddy-current damper is an alternative design. [Upper Story resistor listing](https://store-jp.upperstory.com/collections/spintronic-parts/products/spintronic-resistor-50-1000)

## Shaft and ports

The pitch radius is **20.584 mm** from the reference simulator. All three levels are on one rigid shaft. With signed angular speed $\omega$ and torque into the part $\tau_p$, use $V_p=\tau_p/r$ in N and $I_p=r\omega$ in m/s. Then $V_pI_p=\tau_p\omega$. For kit units, $V_{\rm spin}=10V$ and $I_{\rm spin}=I/10$. Thus resistance and inductance in spin units are 100 times their chain-side SI values. [Mechanism notes, lines 5–11](../../../notes/how-it-works.md#L5) · [Simulator resistor, lines 38–70](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/resistor-part.js#L38) · [Upper Story units](https://upperstory.com/en/spintronics/science)

The grounded housing supplies the reaction. The shaft obeys

$$
\dot\theta=\omega,\qquad J\dot\omega=\sum_p\tau_p-D(\omega).
$$

Each rigid annulus of density $\rho$, thickness $t$, and radii $a,b$ contributes $\int_a^b u^2\rho t\,2\pi u\,du=\pi\rho t(b^4-a^4)/2$ to $J$. The equivalent sprocket and hub add $m_s r^2/2$. Backward Euler solves $J(\omega_1-\omega_0)/h+D(\omega_1)=\sum_p\tau_p$ and advances the angle by $h\omega_1$. The torque solve is bounded by zero and the undamped speed. The oil solve uses bracketed Newton iterations and throws if it fails to converge. The eddy solve is closed form. Multiplying the update by $h\omega_1$ gives input work equal to the change in kinetic energy plus $hD(\omega_1)\omega_1+J(\omega_1-\omega_0)^2/2$. Both terms are nonnegative. [Shaft update](model.js#L115)

`create(params)` starts at rest. `step(h, {top: {torque: 0.006}})` gives the new speed. `step(h, {top: {speed: 10}, middle: {torque: -0.001}})` gives the required top torque with the middle loaded. Speeds are in rad/s and torques in N·m. Omitted ports exert zero torque. A second speed source leaves the reaction split undetermined, so the API rejects it. `ports.top`, `.middle`, and `.bottom` expose `speed`, `torque`, `current`, and `voltage`. [Port implementation](model.js#L154)

## Silicone oil

Assume no slip at the rotor and stationary wall, two axial gaps $g$, and quasi-steady azimuthal Couette flow. At radius $u$, the wall speeds differ by $u\omega$, so the signed shear rate is $\dot\gamma=u\omega/g$. The Carreau law fixes the transition exponent at two. Write $q=\eta_\infty/\eta_0$ and $s=\lambda|\dot\gamma|$.

$$
\eta=\eta_0\big[q+(1-q)(1+s^2)^{(n-1)/2}\big],\qquad
D(\omega)=\frac{4\pi\omega}{g}\int_a^b \eta\,u^3\,du .
$$

Two annular face elements give $4\pi u\,du$. Multiplying shear stress $\eta\dot\gamma$ by that area and lever arm $u$ gives the torque integral. The stress slope contains the factor $(1+ns^2)/(1+s^2)>0$, so $D$ is monotone even as its secant slope falls. The integral uses 16 Simpson intervals. Given label $R_\ell$, set $\omega_*=10V_*/(rR_\ell)$ with $V_*$ in spin V, and solve the equation $D(\omega_*)/\omega_*=R_\ell(r/10)^2$ for $\eta_0$. Nominal $R_\ell$ is therefore a secant resistance at the assumed reference point. Temperature, edge flow, fluid inertia, oil elasticity, seal friction, and stiction are outside this model. [Carreau–Yasuda source, §3.1 Eq. 3](https://www.mdpi.com/2227-9717/9/2/331) · [Oil integration and calibration](model.js#L54)

## Eddy current

For a thin conducting disk of radius $R_d$ and thickness $t$, prescribe normal field $B$ over one circular footprint of radius $a$ centred at $(c,0)$. In the plane, $\mathbf v\times\mathbf B=\omega B(x,y)$ under the pole. The part of this field radial about the pole centre is a gradient with a potential constant on the pole rim. Charge redistribution cancels it. The remaining motional field under the pole is $E_0\hat x$, where $E_0=\omega Bc$. With $\mathbf j=\sigma(-\nabla\phi+\mathbf v\times\mathbf B)$, charge conservation gives $\nabla\cdot\mathbf j=0$ and the insulated disk rim imposes $\mathbf j\cdot\hat n=0$. The current thus scales with $\omega$ at fixed field. [Smythe, *On Eddy Currents in a Rotating Disk*, Eqs. 17–21](https://commons.princeton.edu/motorcycledesign/wp-content/uploads/sites/70/2018/06/eddy_current_brake.pdf)

Set $\mathbf j=(\partial_y\psi,-\partial_x\psi)$ and $d^2=(x-c)^2+y^2$. On an infinite sheet, matching current across the footprint gives $\psi=(\sigma E_0/2)y$ inside and $\psi=(\sigma E_0/2)a^2y/d^2$ outside. Subtract the harmonic image $(\sigma E_0/2)a^2(R_d/c)^2y/[(x-R_d^2/c)^2+y^2]$ to make $\psi=0$ at the rim. Its $y$ derivative averages to its value at the footprint centre by the harmonic mean-value property. The integral of $(x-c,y)\cdot\mathbf j$ over the footprint is zero because it is the angular derivative of $\psi$ integrated around complete circles. Thus the Lorentz torque on the disk is $-Bc t\int j_x\,dA=-D(\omega)$, with

$$
D(\omega)=b_e\omega,\qquad
b_e=\frac{\sigma t B^2\pi a^2 c^2}{2}\left[1-\left(\frac{a R_d}{R_d^2-c^2}\right)^2\right].
$$

The bracket is 0.7854 for the chosen geometry. The drag coefficient is $2.122\times10^{-4}$ N·m·s/rad. The model assumes a fixed, uniform footprint and neglects induced-field feedback, electromagnetic delay, and heating. The 100 Hz sweep therefore characterizes the lumped shaft model. The hardware response at that frequency is unverified. [Smythe, thin-sheet assumptions and boundary construction](https://authors.library.caltech.edu/records/tp1q9-0xk89) · [Eddy model](model.js#L92) · [Bench properties](results.json)

## Parameters

All overrides take positive finite numbers. `exponent` and `viscosityRatio` are at most one. `kind` is `oil` by default or `eddy`. The initial angle and speed are zero by construction. [Model defaults and checks](model.js#L10)

| Parameter | Default | Basis |
| --- | ---: | --- |
| `shaftMass`, both | 0.010 kg | Estimate for a compact sprocket stack, shaft, and hub at equivalent pitch radius. |
| `label`, oil | 200 spin Ω | Listed kit value. The page starts at 50 spin Ω for comparison with eddy. |
| `referenceSpinV`, oil | 3 spin V | Estimate at half the kit’s nominal 6 spin V supply. The factory reference is unverified. |
| `innerRadius`, `outerRadius`, oil | 3, 14 mm | Estimates for hub clearance and a rotor fitting inside the pitch radius. |
| `gap`, oil | 0.30 mm per face | Estimate for a thin gap relative to the rotor. |
| `thickness`, `rotorDensity`, oil | 3 mm, 1,200 kg/m³ | Estimates for a light molded disk. |
| `lambda`, `exponent`, `viscosityRatio`, oil | 6 ms, 0.80, 0.04 | Estimates for a transition at $1/\lambda\approx167$ s⁻¹, mild thinning and a high-shear plateau. |
| `diskRadius`, `thickness`, eddy | 21, 1.5 mm | Estimates for compact thin sheet near sprocket diameter. |
| `poleRadius`, `poleOffset`, eddy | 6, 13 mm | Estimates for an off-axis circular footprint within the disk rim. |
| `field`, eddy | 0.57 T | Estimate selected to give about 50 spin Ω with this geometry. |
| `conductivity`, `rotorDensity`, eddy | 58 MS/m, 8,920 kg/m³ | Copper Development Association C11000 data. |

[Upper Story kit contents and battery label](https://store.upperstory.com/pages/spintronics-act-one-act-two-and-power-pack-includes) · [Copper Development Association, *The Copper Advantage*, Table 4](https://www.copper.org/publications/pub_list/pdf/a1360.pdf)

## Bench

Run `~/.local/bin/node --experimental-default-type=module sim/mech/damper/bench.mjs` from the workspace root. The script applies a signed DC torque sweep through ±6 spin V and fits a line through the origin in V–I space. The step is 0 to 3 spin V, with $t_{63}$ at $1-e^{-1}$ of the settled current. For AC, drive sinusoidal speed at 1% of the 3 V bias current, or 1% of ideal 3 V current at zero bias, and divide the fundamental of voltage by that of current. The least-squares $R+L$ fit uses the backward-Euler derivative $(1-e^{-j2\pi f h})/h$. The raw magnitude and phase are retained. [Bench protocol and checks](bench.mjs) · [All sweeps and transient traces](results.json)

| Mechanism | Ideal R | Fitted DC R | R at 0.25 V | R at 6 V | Current error at 6 V | Max abs. current error | AC R at 3 V | AC L | $t_{63}$ (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Oil, simulator | 20 | 18.00 | 32.05 | 17.09 | +17.03% | 37.59% | 16.29 | 0.5512 | 31.045 |
| Oil, kit | 50 | 44.99 | 70.29 | 42.72 | +17.04% | 28.86% | 40.84 | 0.5512 | 12.316 |
| Oil, kit | 100 | 90.37 | 125.38 | 85.94 | +16.36% | 20.24% | 82.92 | 0.5512 | 6.041 |
| Oil, kit | 200 | 183.55 | 225.86 | 175.58 | +13.91% | 13.91% | 172.99 | 0.5512 | 2.922 |
| Oil, kit | 500 | 479.00 | 517.49 | 466.95 | +7.08% | 7.08% | 471.41 | 0.5512 | 1.123 |
| Oil, kit | 1,000 | 984.34 | 1,010.04 | 974.22 | +2.65% | 2.65% | 981.03 | 0.5512 | 0.555 |
| Oil, simulator | 2,000 | 1,991.03 | 2,005.23 | 1,984.91 | +0.76% | 0.76% | 1,989.65 | 0.5512 | 0.277 |
| Oil, science guide endpoint | 5,000 | 4,996.25 | 5,002.12 | 4,993.65 | +0.13% | 0.13% | 4,995.75 | 0.5512 | 0.111 |
| Eddy | 50 | 50.07 | 50.07 | 50.07 | −0.15% | 0.15% | 50.07 | 1.4647 | 29.251 |

R and L are in spin Ω and spin H. V means spin V. Current error is $100(I/I_{\rm ideal}-1)$ at the same applied voltage. Hardware for the 20 and 2,000 Ω simulator labels and kit inclusion for the 5,000 Ω endpoint are unverified. [Results](results.json) · [Kit contents](https://store.upperstory.com/pages/spintronics-act-one-act-two-and-power-pack-includes) · [Simulator labels](https://github.com/upperstory/spintronics-simulator/blob/251395bb5acad9941f15ea4978266bbe7ee8de00/public/parts/resistor-part.js#L8) · [Science guide range](https://upperstory.com/en/spintronics/science)

| At 3 spin V bias | 0.5 Hz | 10 Hz | 100 Hz |
| --- | ---: | ---: | ---: |
| Ideal 50 Ω | 50 / 0° | 50 / 0° | 50 / 0° |
| Oil 50 Ω | 40.87 / +2.43° | 53.72 / +40.14° | 351.10 / +79.59° |
| Eddy | 50.29 / +5.25° | 105.06 / +61.16° | 924.27 / +83.15° |

Each pair gives impedance magnitude in spin Ω and voltage phase relative to current. At fixed voltage the low labels turn faster, so the oil model reaches higher shear and departs further from its calibration slope. Oil 50 Ω gives 71.24 spin Ω in the zero-bias AC fit and 40.84 spin Ω at 3 spin V despite the 50 Ω secant there. Eddy is linear in the DC model, with a constant 0.15% offset from the 50 Ω comparison and a longer step response from its heavier disk. The two models have zero breakaway torque and respond to ±0.0001 spin V probes. Shaft inertia accounts for the fitted L. [Results](results.json) · [Oil and eddy laws](model.js#L54)

## Integration with `sim/codex`

Import `create`, `PITCH_RADIUS` and `SPIN_SCALE` from `../mech/damper/model.js` in the circuit solver. Attach one instance to each modeled `r` branch when constructing `Circuit`. Pick a hardware label for each part if the preset is meant to represent the kit. The preset values are equivalent-circuit settings. Treat the solver values as spin V and spin A for the adapter below. [Reference settings and presets](../../codex/index.html#L125) · [Reference circuit constructor and stamps](../../codex/index.html#L255)

For a single connected sprocket, let `i0` be the trial current for this branch in spin A and `push` the existing drag voltage in spin V. During `candidate`, linearize the total required branch drop from the last accepted damper state. The companion uses the same `g` and `j` fields as the reference MNA stamp. [Model reaction](model.js#L125) · [Reference companion and candidate](../../codex/index.html#L299)

```js
const speed = i0 * SPIN_SCALE / PITCH_RADIUS;
const { voltage, resistance } = damper.reaction(h, speed);
const v0 = voltage * SPIN_SCALE;
const g = 1 / (resistance * SPIN_SCALE ** 2);
const companion = { g, j: i0 + g * (push - v0), push };
```

Rebuild the companion and solve for updated trial currents until the branch effort residual $v+\mathrm{push}-v_0(i)$ and KCL converge. Oil needs nonlinear iteration. Eddy uses the same affine stamp in one linear solve. Run this within each diode or gate candidate and commit the damper only after the circuit state is accepted, using `damper.step(h, {middle: {speed: i * SPIN_SCALE / PITCH_RADIUS}})`. The damper advances with backward Euler at the circuit’s fixed step. Existing chain rendering can use the branch’s integrated `p.travel`. To render physical shaft angle, use `damper.state.angle` with the existing wrap sign. A circuit using several levels needs equal signed shaft speeds and the sum of signed rim efforts in the shaft balance. [Reference acceptance and travel update](../../codex/index.html#L363) · [Reference chain rendering](../../codex/index.html#L883)

## Verification

Bench assertions passed, including shared-shaft balance, rejection of conflicting drives, decay with no input, the MNA reaction derivative, DC symmetry, calibration, AC inertia and step-size refinement. The first runs exposed a floating-point sample-count assertion and insufficient transient resolution. Both were corrected before generating the saved results. An in-memory adapter test used the reference MNA to drive each damper and matched the standalone torque-driven current, including when drag voltage was applied. All seven reference presets passed their existing checks. [Bench assertions](bench.mjs) · [Reference MNA and tests](../../codex/index.html#L325)

To use the page, serve the directory over HTTP and open `index.html`. A Node smoke test exercised both scenes, the plot and controls using local three.js r177 geometry with a stub renderer and DOM. Browser rendering with pinned r160 is **unverified**. Chromium failed at `setsockopt`, the sandbox blocked opening the local HTTP socket, and jsDelivr DNS resolution failed. [Page imports](index.html#L8)
