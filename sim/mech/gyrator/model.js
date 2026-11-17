// SI at both sprockets: V = torque / radius (N), I = radius * omega (m/s).
// Positive torque and speed use the same shaft sign, so V * I is input power.
export const KIT_CAPACITANCE = 0.12; // m/N, 1.2 spin mF × 100 (notes/how-it-works.md:9,50)
export const KIT_INDUCTANCE = 0.55; // kg, 55 spin H / 100 (notes/how-it-works.md:9,64)
export const defaults = Object.freeze({
  radius: 0.020584,       // m, reference sprocket pitch radius (notes/how-it-works.md:46)
  rotorMass: 0.006,       // kg, estimate for a lightweight disk inside the rings
  rotorRadius: 0.032,     // m, estimate, leaves room around a 64-mm disk for the pivots
  rotorThickness: 0.005, // m, estimate of axial depth for the uniform disk
  spinRate: 285,          // rad/s, estimate chosen for G²C near the nominal kit L
  gimbalInertia1: 1.1e-5, // kg m², estimate: ~9 g effective mass at 35 mm from axis 1
  gimbalInertia2: 7e-6,   // kg m², estimate: ~6 g effective mass at 34 mm from axis 2
  drag1: 2e-5,           // N m s/rad, estimate: weak pivot drag (~2% of H)
  drag2: 3e-5,           // N m s/rad, estimate: weak pivot drag with a higher loss on axis 2
  capacitance: 0,        // m/N, 0 disables the shunt spring; KIT_CAPACITANCE loads port 2
  fixedStep: 1 / 3840,   // s, numerical choice: 60 steps/cycle at 64 Hz
});

function propertiesOf(params) {
  for (const key of Object.keys(params)) {
    if (!Object.hasOwn(defaults, key)) throw new TypeError(`Unknown parameter: ${key}`);
  }
  const p = { ...defaults, ...params };
  for (const [key, value] of Object.entries(p)) {
    const nonnegative = ['drag1', 'drag2', 'capacitance', 'gimbalInertia1', 'gimbalInertia2', 'rotorThickness'].includes(key);
    if (!Number.isFinite(value) || (nonnegative ? value < 0 : key !== 'spinRate' && value <= 0)) {
      throw new RangeError(`Invalid parameter: ${key}=${value}`);
    }
  }
  const r2 = p.radius ** 2;
  const axialInertia = p.rotorMass * p.rotorRadius ** 2 / 2;
  const transverseInertia = p.rotorMass * (3 * p.rotorRadius ** 2 + p.rotorThickness ** 2) / 12;
  const momentum = axialInertia * p.spinRate;
  const j1 = transverseInertia + p.gimbalInertia1;
  const j2 = transverseInertia + p.gimbalInertia2;
  const spring = p.capacitance === 0 ? 0 : r2 / p.capacitance;
  const g = momentum / r2;
  const l1 = j1 / r2;
  const l2 = j2 / r2;
  const r1 = p.drag1 / r2;
  const r2Drag = p.drag2 / r2;
  if (![axialInertia, transverseInertia, momentum, j1, j2, spring, g, l1, l2, r1, r2Drag].every(Number.isFinite)
      || axialInertia <= 0 || l1 <= 0 || l2 <= 0 || (p.capacitance > 0 && spring <= 0)) {
    throw new RangeError('The rotor or port coefficients are outside numeric range');
  }
  return Object.freeze({ ...p, axialInertia, transverseInertia, momentum, j1, j2, spring, g, l1, l2, r1, r2Drag });
}

/**
 * Linearized two-gimbal gyroscope. Port 1 is the outer +x pivot, port 2 the inner
 * +z pivot in the outer frame, and positive rotor spin is about local +y.
 *
 * step(dt, [{speed: rad/s} | {torque: N m}, …]) takes an integer number of
 * fixed ticks. Torque holds; speed prescribes an end rate with a linear ramp
 * from the old rate. Use {torque: 0} for a free port. Each port returns interval
 * means {speed, torque, current, voltage}. state.omega and state.angle hold
 * endpoint values (rad/s, rad). state.energy is gimbal + spring energy (J).
 * With a spring on port 2, the reported effort is the additional external effort.
 */
export function create(params = {}) {
  const properties = propertiesOf(params);
  const { radius, fixedStep: h, spinRate, j1, j2, momentum: H, spring: k, drag1, drag2 } = properties;
  const angle = [0, 0];
  const omega = [0, 0];
  const state = { angle, omega, time: 0, spinAngle: 0, energy: 0 };
  const ports = Array.from({ length: 2 }, () => ({ speed: 0, torque: 0, current: 0, voltage: 0 }));

  // (J Δω/h) + (D + gyro) ω_mid + (0, k θ2_mid) = τ.
  // θ2_mid = θ2_old + h ω2_mid/2. Solve the two midpoint rates together.
  const a = 2 * j1 / h + drag1;
  const d = 2 * j2 / h + drag2 + k * h / 2;
  const determinant = a * d + H * H;
  if (!Number.isFinite(determinant) || determinant <= 0) {
    throw new RangeError('The midpoint system is outside numeric range');
  }

  function step(dt, drive) {
    const ticks = Math.round(dt / h);
    if (!Number.isSafeInteger(ticks) || ticks < 1 || Math.abs(dt / h - ticks) > 1e-9) {
      throw new RangeError('dt must be a positive integer multiple of fixedStep');
    }
    if (!Array.isArray(drive) || drive.length !== 2 || ![drive[0], drive[1]].every(command =>
      command && Object.keys(command).length === 1
      && ((Object.hasOwn(command, 'speed') && Number.isFinite(command.speed))
        || (Object.hasOwn(command, 'torque') && Number.isFinite(command.torque))))) {
      throw new TypeError('Supply {speed} or {torque} for each port');
    }
    const [first, second] = drive;
    const prescribed1 = Object.hasOwn(first, 'speed');
    const prescribed2 = Object.hasOwn(second, 'speed');
    const from1 = omega[0];
    const from2 = omega[1];
    const rateChange1 = prescribed1 ? first.speed - from1 : 0;
    const rateChange2 = prescribed2 ? second.speed - from2 : 0;
    let w1 = from1, w2 = from2, q1 = angle[0], q2 = angle[1];
    let speedSum1 = 0, speedSum2 = 0, torqueSum1 = 0, torqueSum2 = 0;
    for (let tick = 0; tick < ticks; tick++) {
      const history1 = 2 * j1 / h * w1;
      const history2 = 2 * j2 / h * w2 - k * q2;
      const fraction = (tick + 0.5) / ticks;
      let mid1, mid2;
      if (prescribed1) {
        mid1 = from1 + fraction * rateChange1;
        mid2 = prescribed2 ? from2 + fraction * rateChange2 : (history2 + second.torque - H * mid1) / d;
      } else if (prescribed2) {
        mid2 = from2 + fraction * rateChange2;
        mid1 = (history1 + first.torque + H * mid2) / a;
      } else {
        const b1 = history1 + first.torque;
        const b2 = history2 + second.torque;
        mid1 = (d * b1 + H * b2) / determinant;
        mid2 = (a * b2 - H * b1) / determinant;
      }
      torqueSum1 += prescribed1 ? a * mid1 - H * mid2 - history1 : first.torque;
      torqueSum2 += prescribed2 ? d * mid2 + H * mid1 - history2 : second.torque;
      speedSum1 += mid1;
      speedSum2 += mid2;
      w1 = 2 * mid1 - w1;
      w2 = 2 * mid2 - w2;
      q1 += h * mid1;
      q2 += h * mid2;
    }

    const energy = (j1 * w1 ** 2 + j2 * w2 ** 2 + k * q2 ** 2) / 2;
    const time = state.time + ticks * h;
    // The axial momentum is fixed. The first-order spin phase is Ωt.
    const spinAngle = (state.spinAngle + spinRate * ticks * h) % (2 * Math.PI);
    const readings = [
      [speedSum1 / ticks, torqueSum1 / ticks],
      [speedSum2 / ticks, torqueSum2 / ticks],
    ].map(([speed, torque]) => ({ speed, torque, current: radius * speed, voltage: torque / radius }));
    if (![energy, time, spinAngle, w1, w2, q1, q2, ...readings.flatMap(Object.values)].every(Number.isFinite)) {
      throw new RangeError('Non-finite gyrator step');
    }
    angle[0] = q1;
    angle[1] = q2;
    omega[0] = w1;
    omega[1] = w2;
    Object.assign(state, { time, spinAngle, energy });
    ports.forEach((port, index) => Object.assign(port, readings[index]));
    return ports;
  }

  return { state, step, ports, properties };
}
