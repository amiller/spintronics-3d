// SI port variables are torque [N m] and angular speed [rad/s]. The chain
// equivalents are effort V = torque/r [N] and flow I = r*speed [m/s].
export const STEP = 1 / 480; // s, matches sim/codex/index.html:127
const gateRadius = 0.063772 / 2;
const channelRadius = 0.03398 / 2;
const fullGateEffort = 0.15; // N, Upper Story's approximate 1.5 spin V
const gateTravel = 0.022 / gateRadius; // rad, from the listed 2.2 mspin C
const clamp = (n, lo, hi) => Math.max(lo, Math.min(n, hi));

// Estimates are explained with their derivations and source links in REPORT.md.
export const defaults = Object.freeze({
  mechanism: 'spring', // 'spring' (kit reconstruction) or 'governor' (alternative)
  gateRadius, // m, simulator pitch radius (notes/how-it-works.md:93)
  channelRadius, // m, simulator pitch radius (notes/how-it-works.md:93)
  gateTravel, // rad, catalog gate charge / pitch radius
  fullGateEffort, // N, catalog full-open gate voltage in chain units
  gateDirection: 1, // +1 or -1, reversible gate polarity per Upper Story
  gateInertia: 7e-6, // kg m², estimate: 14 g equivalent disk at gate radius
  gateDrag: 2.5e-4, // N m s/rad, estimate: moderate spring return damping
  channelInertia: 2e-6, // kg m², estimate: 14 g equivalent disk at channel radius
  channelDrag: 2e-4, // N m s/rad, estimate: small running loss with pads clear
  drumRadius: 0.012, // m, estimate: brake ring inside channel pitch radius
  padTravel: 0.0006, // m, estimate: sub-mm elastic pad release
  padStiffness: 5500, // N/m, combined estimate anchored to early sample drag
  muStatic: 0.65, // dimensionless, estimate: breakaway above sliding friction
  muKinetic: 0.50, // dimensionless, estimate: friction coefficient scale
  flyballMass: 0.010, // kg per ball, design estimate for a compact pair
  flyballRadius: 0.020, // m, design estimate for inward ball-center radius
  flyballStroke: 0.003, // m, design estimate for ball and linkage clearance
  governorHubInertia: 4e-6, // kg m², estimate: 8 g equivalent disk at gate radius
  governorDrag: 1.2e-4, // N m s/rad, estimate: light gate shaft running loss
  governorPreload: 0.035, // N, design estimate for an inward return preload
  governorStiffness: 28, // N/m, design estimate for the combined return spring
  governorDamping: 0.9, // N s/m, estimate: damped radial motion
});

function parameters(overrides) {
  for (const key of Object.keys(overrides)) {
    if (!Object.hasOwn(defaults, key)) throw new TypeError('Unknown parameter ' + key);
  }
  const p = {...defaults, ...overrides};
  if (!['spring', 'governor'].includes(p.mechanism)) throw new RangeError('Unknown mechanism');
  if (p.gateDirection !== 1 && p.gateDirection !== -1) throw new RangeError('gateDirection must be +1 or -1');
  const nonnegative = ['gateDrag', 'muStatic', 'muKinetic', 'governorDrag', 'governorPreload', 'governorDamping'];
  for (const [key, value] of Object.entries(p)) {
    if (key === 'mechanism' || key === 'gateDirection') continue;
    if (!Number.isFinite(value) || value < 0 || (value === 0 && !nonnegative.includes(key))) {
      throw new RangeError('Invalid parameter ' + key);
    }
  }
  if (p.muStatic < p.muKinetic) throw new RangeError('muStatic must be at least muKinetic');
  const camLead = p.padTravel / p.gateTravel;
  if (p.mechanism === 'spring' && p.fullGateEffort * p.gateRadius <= p.padStiffness * p.padTravel * camLead) {
    throw new RangeError('Cam load leaves no positive return-spring stiffness');
  }
  return p;
}

function validateDrive(input, name) {
  if (!input || typeof input !== 'object' || Object.keys(input).length !== 1 ||
      !((Object.hasOwn(input, 'speed') && Number.isFinite(input.speed)) ||
        (Object.hasOwn(input, 'torque') && Number.isFinite(input.torque)))) {
    throw new TypeError(name + ' requires exactly one finite speed or torque');
  }
}

function springGate(p, previous, drive, h) {
  const {angle, speed} = previous;
  const s = p.gateDirection;
  const q = s * angle;
  const w = s * speed;
  const k = p.fullGateEffort * p.gateRadius / p.gateTravel;
  const effectiveInertia = p.gateInertia / h + p.gateDrag;
  if (Object.hasOwn(drive, 'speed')) {
    const qNext = q + h * s * drive.speed;
    const tolerance = 16 * Number.EPSILON * p.gateTravel;
    if (qNext < -tolerance || qNext > p.gateTravel + tolerance) {
      throw new RangeError('Gate speed would cross a hard stop');
    }
    const position = clamp(qNext, 0, p.gateTravel);
    const torque = p.gateInertia * (drive.speed - speed) / h + p.gateDrag * drive.speed + s * k * position;
    return {angle: s * position, speed: drive.speed, torque, opening: position / p.gateTravel};
  }

  const freeSpeed = (p.gateInertia * w / h + s * drive.torque - k * q) / (effectiveInertia + h * k);
  const position = clamp(q + h * freeSpeed, 0, p.gateTravel);
  const nextSpeed = position === q + h * freeSpeed ? freeSpeed : (position - q) / h;
  return {angle: s * position, speed: s * nextSpeed, torque: drive.torque, opening: position / p.gateTravel};
}

function governorGate(p, previous, drive, h) {
  const {angle, speed, flyballPosition: x, flyballSpeed: u} = previous;
  const m = 2 * p.flyballMass;
  const radius = p.flyballRadius + x;
  const inertia = p.governorHubInertia + m * radius ** 2;
  const speedDriven = Object.hasOwn(drive, 'speed');
  const drivenSpeed = speedDriven ? drive.speed : (inertia * speed + h * drive.torque) / (inertia + h * p.governorDrag);
  const radialInertia = m / h;
  const lead = p.padTravel / p.flyballStroke;
  const k = p.governorStiffness + p.padStiffness * lead ** 2;
  const force = m * radius * drivenSpeed ** 2 - p.governorPreload - k * x;
  const freeSpeed = (radialInertia * u + force) / (radialInertia + p.governorDamping + h * k);
  const position = clamp(x + h * freeSpeed, 0, p.flyballStroke);
  const radialSpeed = position === x + h * freeSpeed ? freeSpeed : (position - x) / h;
  const nextInertia = p.governorHubInertia + m * (p.flyballRadius + position) ** 2;
  const nextSpeed = speedDriven ? drivenSpeed : inertia * drivenSpeed / nextInertia;
  const torque = speedDriven
    ? (nextInertia * nextSpeed - inertia * speed) / h + p.governorDrag * nextSpeed
    : drive.torque;
  return {
    angle: angle + h * nextSpeed, speed: nextSpeed, torque, opening: 1 - position / p.flyballStroke,
    flyballPosition: position, flyballSpeed: radialSpeed,
  };
}

function channelStep(p, previous, drive, normal, h) {
  const {angle, speed} = previous;
  const J = p.channelInertia;
  const b = p.channelDrag;
  const staticLimit = p.muStatic * normal * p.drumRadius;
  const kineticLimit = p.muKinetic * normal * p.drumRadius;
  let nextSpeed;
  let torque;
  if (Object.hasOwn(drive, 'speed')) {
    nextSpeed = drive.speed;
    const slidingSign = Math.sign(nextSpeed || speed);
    torque = J * (nextSpeed - speed) / h + b * nextSpeed + kineticLimit * slidingSign;
  } else {
    torque = drive.torque;
    const oldSign = Math.sign(speed);
    const sliding = (J * speed + h * (torque - kineticLimit * oldSign)) / (J + h * b);
    if (oldSign * sliding > 0) {
      nextSpeed = sliding;
    } else if (Math.abs(torque) <= staticLimit) {
      nextSpeed = 0;
    } else {
      // Use the remaining time after zero crossing to launch in the new direction.
      const stopTime = oldSign ? J * Math.abs(speed) / (kineticLimit - oldSign * torque) : 0;
      const remaining = Math.max(0, h - stopTime);
      nextSpeed = remaining * (torque - kineticLimit * Math.sign(torque)) / (J + remaining * b);
    }
  }
  return {angle: angle + h * nextSpeed, speed: nextSpeed, torque,
    stuck: nextSpeed === 0 && normal > 0, staticLimit, kineticLimit};
}

function port(radius, {speed, torque}) {
  const V = torque / radius;
  const I = radius * speed;
  if (!Number.isFinite(V) || !Number.isFinite(I)) throw new RangeError('Port overflow');
  return Object.freeze({radius, speed, torque, V, I});
}

/**
 * create(params) returns state and port snapshots, plus step(h, drive).
 * h must be STEP; repeat calls to advance longer. Supply both gate and channel
 * as {torque: N_m} or {speed: rad_per_s}. V and I are signed effort/flow into
 * the part. The snapshots reflect the most recent accepted end-of-step state.
 * preview(h, drive) evaluates the same update without committing history.
 */
export function create(overrides = {}) {
  const p = parameters(overrides);
  const spring = p.mechanism === 'spring';
  const initial = {angle: 0, speed: 0, torque: 0};
  const initialNormal = spring ? p.padStiffness * p.padTravel : 0;
  const initialGate = spring ? {...initial, opening: 0} :
    {...initial, opening: 1, flyballPosition: 0, flyballSpeed: 0};

  function snapshot(time, gate, channel, normalForce) {
    for (const value of [gate.angle, gate.flyballSpeed ?? 0, channel.angle, normalForce, channel.staticLimit]) {
      if (!Number.isFinite(value)) throw new RangeError('State overflow');
    }
    return {
      state: Object.freeze({time, normalForce, gate: Object.freeze(gate), channel: Object.freeze(channel)}),
      ports: Object.freeze({gate: port(p.gateRadius, gate), channel: port(p.channelRadius, channel)}),
    };
  }

  let current = snapshot(0, initialGate, channelStep(p, initial, {torque: 0}, initialNormal, STEP), initialNormal);
  function preview(h, drive) {
    if (h !== STEP) throw new RangeError('Use the exported fixed step STEP');
    validateDrive(drive?.gate, 'gate');
    validateDrive(drive?.channel, 'channel');
    const {state} = current;
    const gate = spring ? springGate(p, state.gate, drive.gate, h) : governorGate(p, state.gate, drive.gate, h);
    const fraction = spring ? 1 - gate.opening : gate.flyballPosition / p.flyballStroke;
    const normal = p.padStiffness * p.padTravel * fraction;
    const channel = channelStep(p, state.channel, drive.channel, normal, h);
    return snapshot(state.time + h, gate, channel, normal);
  }

  return {
    get state() { return current.state; },
    get ports() { return current.ports; },
    preview,
    step(h, drive) {
      current = preview(h, drive);
      return current.ports;
    },
  };
}
