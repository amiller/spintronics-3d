/**
 * One rigid shaft, three coaxial chain ports. Chain-side SI: V = torque/r [N],
 * I = r*speed [m/s]. Positive torque and speed deliver power into the part.
 * Drive keys are top, middle and bottom. Each takes {speed: rad/s} OR
 * {torque: N m}. Omitted keys exert zero torque. One speed constraint at most.
 * `ports` reports {speed, torque, current, voltage} at the end of each step.
 * `state` reports {time [s], angle [rad], omega [rad/s], dragTorque [N m]}.
 * Keep dt [s] fixed at the caller. See REPORT.md for sources and equations.
 */
export const PITCH_RADIUS = 0.020584; // m, notes/how-it-works.md:46 (simulator radius)
export const SPIN_SCALE = 10; // m/spin C, notes/how-it-works.md:9
export const KIT_LABELS = Object.freeze([50, 100, 200, 500, 1000]); // Upper Story kit contents
export const SIM_LABELS = Object.freeze([20, ...KIT_LABELS, 2000]); // resistor-part.js:8
export const SCIENCE_MAX_LABEL = 5000; // Upper Story science guide, resistor range endpoint
const PORT_NAMES = ['top', 'middle', 'bottom'];

export const COMMON = Object.freeze({
  // Estimate: equivalent mass of a compact sprocket stack, shaft and hub,
  // lumped into a solid disk at the pitch radius to compute shaft inertia.
  shaftMass: 0.010, // kg
});
export const OIL = Object.freeze({
  label: 200,           // spin Ω, a hardware label listed by Upper Story
  referenceSpinV: 3,   // spin V, assumed midpoint of the nominal 6 spin V supply
  innerRadius: 0.003,  // m, estimated shaft/hub clearance
  outerRadius: 0.014,  // m, estimated to fit inside the pitch radius
  gap: 0.00030,        // m, estimated thin oil gap on each face of the disk
  thickness: 0.003,   // m, estimated rigid disk thickness
  rotorDensity: 1200, // kg/m³, estimate for a lightweight molded rotor
  lambda: 0.006,      // s, estimate putting the outer-face shear transition near 170 s⁻¹
  exponent: 0.80,     // dimensionless, estimate for mild shear thinning
  viscosityRatio: 0.04, // dimensionless etaInfinity/etaZero, estimated high-shear plateau
});
export const EDDY = Object.freeze({
  diskRadius: 0.021,  // m, estimate for a disk near the sprocket diameter
  thickness: 0.0015, // m, estimate for thin sheet
  poleRadius: 0.006,  // m, estimate for one compact circular flux footprint
  poleOffset: 0.013,  // m, estimate placing it off-axis and inside the disk
  field: 0.57,        // T, estimated gap field selected for a roughly 50 spin Ω design
  conductivity: 58e6, // S/m, Copper Development Association, C11000, Table 4
  rotorDensity: 8920, // kg/m³, same source, C11000
});

function finite(name, value) {
  if (!Number.isFinite(value)) throw new RangeError(`${name} must be finite`);
  return value;
}

function positive(name, value) {
  if (!(finite(name, value) > 0)) throw new RangeError(`${name} must be positive`);
  return value;
}

function oil(config) {
  const { label, referenceSpinV, innerRadius, outerRadius, gap, thickness,
    rotorDensity, lambda, exponent, viscosityRatio } = config;
  if (innerRadius >= outerRadius) throw new RangeError('Oil annulus must have positive width');
  if (exponent > 1 || viscosityRatio > 1) throw new RangeError('Carreau ratios must be at most one');

  // Composite Simpson rule for two axial gaps, no slip. Radial nodes are fixed.
  const intervals = 16;
  const dr = (outerRadius - innerRadius) / intervals;
  const rings = Array.from({ length: intervals + 1 }, (_, index) => {
    const radius = innerRadius + index * dr;
    const weight = index === 0 || index === intervals ? 1 : index % 2 ? 4 : 2;
    return { shearScale: lambda * radius / gap,
      geometry: 4 * Math.PI * radius ** 3 / gap * weight * dr / 3 };
  });
  const power = (exponent - 1) / 2;
  function shearIntegral(speed, derivative = false) {
    let total = 0;
    for (const { shearScale, geometry } of rings) {
      const squaredShear = (shearScale * speed) ** 2;
      const base = 1 + squaredShear;
      const relative = (1 - viscosityRatio) * base ** power;
      const factor = derivative ? 1 + (exponent - 1) * squaredShear / base : 1;
      total += geometry * (viscosityRatio + relative * factor);
    }
    return total;
  }
  const referenceSpeed = referenceSpinV * SPIN_SCALE / (label * PITCH_RADIUS);
  const targetAngularDrag = label * (PITCH_RADIUS / SPIN_SCALE) ** 2;
  const zeroShearViscosity = targetAngularDrag / shearIntegral(referenceSpeed);
  const rotorInertia = Math.PI / 2 * rotorDensity * thickness * (outerRadius ** 4 - innerRadius ** 4);
  return {
    rotorInertia, zeroShearViscosity, referenceSpeed,
    torque: speed => zeroShearViscosity * speed * shearIntegral(speed),
    slope: speed => zeroShearViscosity * shearIntegral(speed, true),
  };
}

function eddy(config) {
  const { diskRadius, thickness, poleRadius, poleOffset, field, conductivity, rotorDensity } = config;
  if (poleRadius + poleOffset >= diskRadius) throw new RangeError('Pole must fit inside the copper disk');
  const boundaryFactor = 1 - (poleRadius * diskRadius / (diskRadius ** 2 - poleOffset ** 2)) ** 2;
  const coefficient = conductivity * thickness * field ** 2 * Math.PI * poleRadius ** 2 *
    poleOffset ** 2 * boundaryFactor / 2;
  const rotorInertia = Math.PI / 2 * rotorDensity * thickness * diskRadius ** 4;
  return { rotorInertia, coefficient, boundaryFactor, torque: speed => coefficient * speed,
    slope: () => coefficient };
}

export function create(params = {}) {
  if (params === null || typeof params !== 'object' || Array.isArray(params)) {
    throw new TypeError('Expected a damper parameter object');
  }
  const { kind = 'oil', ...overrides } = params;
  if (kind !== 'oil' && kind !== 'eddy') throw new RangeError(`Unknown damper ${kind}`);
  const config = { kind, ...COMMON, ...(kind === 'oil' ? OIL : EDDY) };
  for (const [name, value] of Object.entries(overrides)) {
    if (!Object.hasOwn(config, name)) throw new RangeError(`Unknown parameter ${name} for ${kind}`);
    config[name] = positive(name, value);
  }

  const law = kind === 'oil' ? oil(config) : eddy(config);
  const inertia = positive('shaft inertia', law.rotorInertia + config.shaftMass * PITCH_RADIUS ** 2 / 2);
  const inductance = inertia / PITCH_RADIUS ** 2; // N per (m/s²), i.e. kg
  const state = { time: 0, angle: 0, omega: 0, dragTorque: 0 };
  const ports = Object.fromEntries(PORT_NAMES.map(name => [name, { speed: 0, torque: 0, current: 0, voltage: 0 }]));

  function steady(speed) {
    return finite('steady torque', law.torque(finite('speed', speed)));
  }

  // Non-mutating single-shaft reaction for implicit circuit coupling.
  // resistance is the discrete d(voltage)/d(current), in N s/m.
  function reaction(dt, speed) {
    positive('dt', dt);
    const torque = finite('reaction torque', inertia * (speed - state.omega) / dt + steady(speed));
    const resistance = positive('reaction slope', inductance / dt + law.slope(speed) / PITCH_RADIUS ** 2);
    return { torque, voltage: torque / PITCH_RADIUS, resistance };
  }

  function drivenSpeed(massRate, appliedTorque) {
    const freeSpeed = finite('undamped speed', state.omega + appliedTorque / massRate);
    if (kind === 'eddy') return freeSpeed / (1 + law.coefficient / massRate);
    // F(w) = massRate*(w-freeSpeed) + drag(w) is strictly increasing.
    // Zero and the undamped speed always bracket the solution.
    let low = Math.min(0, freeSpeed);
    let high = Math.max(0, freeSpeed);
    let speed = freeSpeed / (1 + law.slope(0) / massRate);
    for (let iteration = 0; iteration < 80; iteration++) {
      const drag = steady(speed);
      const residual = massRate * (speed - freeSpeed) + drag;
      if (Math.abs(residual) <= 1e-12 * Math.max(Math.abs(drag), Math.abs(massRate * freeSpeed), 1e-30)) return speed;
      if (residual < 0) low = speed;
      else high = speed;
      const next = speed - residual / (massRate + law.slope(speed));
      speed = next > low && next < high ? next : low + (high - low) / 2;
    }
    throw new Error('Implicit oil step failed to converge');
  }

  function step(dt, drive = {}) {
    positive('dt', dt);
    if (drive === null || typeof drive !== 'object' || Array.isArray(drive)) {
      throw new TypeError('Expected a drive object keyed by sprocket');
    }
    let imposedPort;
    let speed;
    let suppliedTorque = 0;
    for (const [name, input] of Object.entries(drive)) {
      if (!Object.hasOwn(ports, name)) throw new RangeError(`Unknown port ${name}`);
      if (input === null || typeof input !== 'object' || Object.keys(input).length !== 1) {
        throw new TypeError(`${name} requires one speed or torque`);
      }
      if (Object.hasOwn(input, 'speed')) {
        if (imposedPort) throw new RangeError('A rigid shaft accepts only one imposed speed');
        imposedPort = name;
        speed = finite(`${name} speed`, input.speed);
      } else if (Object.hasOwn(input, 'torque')) {
        suppliedTorque += finite(`${name} torque`, input.torque);
      } else {
        throw new TypeError(`${name} requires speed or torque`);
      }
    }
    const massRate = positive('inertia/dt', inertia / dt);
    speed = imposedPort ? speed : drivenSpeed(massRate, finite('total torque', suppliedTorque));
    const dragTorque = steady(speed);
    const holdingTorque = imposedPort
      ? finite('holding torque', massRate * (speed - state.omega) + dragTorque - suppliedTorque) : 0;
    const current = finite('current', PITCH_RADIUS * speed);
    const time = finite('time', state.time + dt);
    const angle = finite('angle', state.angle + dt * speed);
    const readings = PORT_NAMES.map(name => {
      const torque = name === imposedPort ? holdingTorque : drive[name]?.torque ?? 0;
      return { speed, torque, current, voltage: finite('voltage', torque / PITCH_RADIUS) };
    });
    Object.assign(state, { time, angle, omega: speed, dragTorque });
    PORT_NAMES.forEach((name, index) => Object.assign(ports[name], readings[index]));
    return ports;
  }

  const properties = Object.freeze({ ...config, inertia, inductance, rotorInertia: law.rotorInertia,
    ...(kind === 'oil' ? { zeroShearViscosity: law.zeroShearViscosity, referenceSpeed: law.referenceSpeed }
      : { coefficient: law.coefficient, boundaryFactor: law.boundaryFactor }) });
  return { state, ports, step, steady, reaction, properties };
}
