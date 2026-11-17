// SI ports: voltage = torque / radius (N), current = radius * angular speed (m/s).
// Parameter provenance, reductions, and integration equations are in REPORT.md.
const TAU = 2 * Math.PI;
const CAP_RADIUS = 0.041168 / 2;
const SMALL_RADIUS = 0.025771 / 2;
export const KIT = Object.freeze({capacitance: 0.12, inductance: 0.55, voltage: 0.6, travel: 10.8});
export const CHAIN = Object.freeze({
  pitch: 0.0064, links: 25, linkMass: 0.00012, stiffness: 2000,
  damping: 0.08, linkDrag: 0.00015, preload: 0.45, polygonal: true,
});
const DEFAULT_STEP = 1 / 24000;
const CAP_K = CAP_RADIUS ** 2 / KIT.capacitance;
export const PARTS = Object.freeze(Object.fromEntries(Object.entries({
  capacitor: {
    radius: CAP_RADIUS, teeth: 20, inertia: 2e-6, drag: 2e-5, friction: 2.5e-5,
    spring: CAP_K, hysteresisK: 0.002, hysteresisTorque: 3e-5,
  },
  flywheel: {radius: SMALL_RADIUS, teeth: 13, inertia: KIT.inductance * SMALL_RADIUS ** 2, drag: 1e-5, friction: 1e-5},
  motor: {
    radius: SMALL_RADIUS, teeth: 13, inertia: 6e-6, drag: 5e-6, friction: 1e-5,
    motorTorque: KIT.voltage * SMALL_RADIUS, capacity: KIT.travel / SMALL_RADIUS,
    tripSpeed: 20 * 39 / 5, brakeDrag: 0.003,
  },
  idler: {radius: SMALL_RADIUS, teeth: 13, inertia: 2e-6, drag: 0, friction: 0},
  inerter: {
    radius: SMALL_RADIUS, teeth: 13, inertia: 2e-6, drag: 2e-6, friction: 1e-5,
    lead: 0.008, ratio: 1, screwInertia: 0.87e-6, screwDrag: 8e-8,
  },
}).map(([key, value]) => [key, Object.freeze(value)])));

const clamp = (x, lo, hi) => Math.max(lo, Math.min(hi, x));

function numeric(name, value, min, strict = false) {
  if (!Number.isFinite(value) || value < min || (strict && value === min)) {
    throw new RangeError(`${name} must be finite and ${strict ? '>' : '>='} ${min}`);
  }
}

function validated(defaults, overrides, additional = []) {
  if (overrides === null || typeof overrides !== 'object' || Array.isArray(overrides)) {
    throw new TypeError('Parameters must be an object');
  }
  for (const key of Object.keys(overrides)) {
    if (!Object.hasOwn(defaults, key) && !additional.includes(key)) throw new TypeError(`Unknown parameter ${key}`);
  }
  return {...defaults, ...overrides};
}

// A tooth advances one pitch over one angular sector. `radius` is the reference
// for nominal ports. `pitchRadius` is the polygon's circumradius from p and N.
export function engagement(angle, teeth, pitch) {
  const half = Math.PI / teeth;
  const sector = Math.floor((angle + half) / (2 * half));
  const phase = angle - sector * (2 * half);
  const pitchRadius = pitch / (2 * Math.sin(half));
  return {
    travel: sector * pitch + pitchRadius * Math.sin(phase),
    arm: pitchRadius * Math.cos(phase),
    slope: -pitchRadius * Math.sin(phase),
  };
}

function addPart(spec) {
  if (typeof spec === 'string') spec = {kind: spec};
  const {kind} = spec;
  if (!Object.hasOwn(PARTS, kind)) throw new TypeError(`Unknown part ${kind}`);
  const p = validated(PARTS[kind], spec, ['kind', 'angle', 'speed']);
  for (const [key, value] of Object.entries(p)) {
    if (key === 'kind') continue;
    if (key === 'angle' || key === 'speed') {
      if (!Number.isFinite(value)) throw new TypeError(`${key} must be finite`);
    } else numeric(key, value, 0, ['radius', 'inertia', 'lead', 'ratio', 'tripSpeed'].includes(key));
  }
  if (!Number.isInteger(p.teeth) || p.teeth < 3) throw new RangeError('teeth must be an integer >= 3');
  const gear = kind === 'inerter' ? TAU * p.ratio * p.radius / p.lead : 0;
  return {
    p: Object.freeze(p), gear, inertia: p.inertia + (p.screwInertia ?? 0) * gear ** 2,
    drag: p.drag + (p.screwDrag ?? 0) * gear ** 2,
    angle: p.angle ?? 0, omega: p.speed ?? 0, hysteresis: 0,
    tripped: false, remaining: p.capacity ?? 0, feed: 0, feedSpeed: 0,
    chainTorque: 0, flywheelAngle: gear * (p.angle ?? 0),
  };
}

function configureChain(spec) {
  const chain = validated(CHAIN, spec);
  for (const [key, value] of Object.entries(chain)) {
    if (key === 'polygonal') {
      if (typeof value !== 'boolean') throw new TypeError('polygonal must be boolean');
    } else numeric(key, value, 0, ['pitch', 'linkMass', 'stiffness'].includes(key));
  }
  if (!Number.isInteger(chain.links) || chain.links < 2) throw new RangeError('links per span must be an integer >= 2');
  return Object.freeze(chain);
}

function limb(n, start, end, preload) {
  return {
    start, end, displacement: new Float64Array(n + 1), speed: new Float64Array(n + 1),
    tension: new Float64Array(n).fill(preload),
  };
}

function chord(shaft, chain) {
  return chain.polygonal ? engagement(shaft.angle, shaft.p.teeth, chain.pitch)
    : {travel: shaft.p.radius * shaft.angle, arm: shaft.p.radius, slope: 0};
}

function tension(chain, dx, dv) {
  const elastic = chain.preload + chain.stiffness * dx;
  return elastic > 0 ? Math.max(0, elastic + chain.damping * dv) : 0;
}

function loadLimb(span, chain, shafts) {
  const {displacement: x, speed: v, tension: force, start, end} = span;
  x[0] = shafts[start].feed;
  x[chain.links] = shafts[end].feed;
  v[0] = shafts[start].feedSpeed;
  v[chain.links] = shafts[end].feedSpeed;
  for (let j = 0; j < chain.links; j++) force[j] = tension(chain, x[j + 1] - x[j], v[j + 1] - v[j]);
}

function advanceLimb(span, chain, h) {
  const {displacement: x, speed: v, tension: force} = span;
  for (let j = 1; j < chain.links; j++) {
    v[j] += h * (force[j] - force[j - 1] - chain.linkDrag * v[j]) / chain.linkMass;
    x[j] += h * v[j];
  }
}

// Each branch expresses applied torque = slope * next angular speed + intercept.
// The middle entry is a set-valued Coulomb law at zero speed.
function shaftLaw(shaft, h, inertia, drag, bias) {
  const {p, omega, angle, hysteresis} = shaft;
  const k = p.spring ?? 0;
  const kh = p.hysteresisK ?? 0;
  const limit = kh ? p.hysteresisTorque ?? 0 : 0;
  const limitSpeed = limit && kh ? limit / (kh * h) : 0;
  const hystSpeed = limitSpeed ? hysteresis / (kh * h) : 0;
  const base = inertia / h + drag + k * h;
  const offset = k * angle - inertia * omega / h + bias;
  return {base, offset, khStep: limitSpeed ? kh * h : 0, limit, hysteresis, lo: -limitSpeed - hystSpeed,
    hi: limitSpeed - hystSpeed, friction: p.friction};
}

function torqueAt(law, omega) {
  const {base, offset, khStep, limit, hysteresis, friction} = law;
  return base * omega + offset + clamp(hysteresis + khStep * omega, -limit, limit) + friction * Math.sign(omega);
}

function speedAt(law, torque) {
  const {base, offset, khStep, limit, hysteresis, lo, hi, friction} = law;
  const relative = torque - offset - hysteresis;
  if (Math.abs(relative) <= friction) return 0;
  const target = torque - offset - friction * Math.sign(relative);
  const linear = (target - hysteresis) / (base + khStep);
  if (linear < lo) return (target + limit) / base;
  if (linear > hi) return (target - limit) / base;
  return linear;
}

function motorTorque(shaft) {
  return shaft.p.kind === 'motor' && !shaft.tripped && shaft.remaining > 0 ? shaft.p.motorTorque : 0;
}

/**
 * create({left, right, chain, h}) -> {state, step(dt, drive), ports, rewind, companion}.
 * Parts are names or overrides with `kind`. Use right:null for a single shaft.
 * `drive` is keyed by left/right. A command is {torque: N*m} or {speed: rad/s}.
 * Omitted commands impose zero external torque. A callback (endTime, ports)
 * may supply commands every substep. Each dt must be an integer multiple of h.
 * `ports` reports last-step applied torque or speed, plus their nominal SI analogs.
 */
export function create(params = {}) {
  const options = validated({left: 'capacitor', right: 'flywheel', chain: {}, h: DEFAULT_STEP}, params);
  const {h} = options;
  numeric('h', h, 0, true);
  const shafts = {left: addPart(options.left)};
  if (options.right !== null) shafts.right = addPart(options.right);
  if (!shafts.right && Object.keys(options.chain).length) throw new TypeError('Chain requires two shafts');
  const chain = shafts.right ? configureChain(options.chain) : null;
  const strands = chain ? [limb(chain.links, 'left', 'right', chain.preload), limb(chain.links, 'right', 'left', chain.preload)] : [];
  const ports = {};
  const entries = Object.entries(shafts);
  const initialTravel = {};
  for (const [id, shaft] of entries) {
    const {p} = shaft;
    ports[id] = {radius: p.radius, angle: shaft.angle, speed: shaft.omega, torque: null,
      current: p.radius * shaft.omega, voltage: null, chainTorque: 0, feedSpeed: null};
    if (!chain) continue;
    const geometry = chord(shaft, chain);
    const armMax = chain.polygonal ? chain.pitch / (2 * Math.sin(Math.PI / p.teeth)) : p.radius;
    const massFactor = armMax ** 2 / (shaft.inertia + chain.linkMass * armMax ** 2);
    const bound = 4 * Math.max(1 / chain.linkMass, massFactor);
    if (bound * (h * h * chain.stiffness + 2 * h * (chain.damping + chain.linkDrag)) >= 3) {
      throw new RangeError('Chain step too large for this link mass, stiffness and damping');
    }
    initialTravel[id] = geometry.travel;
    shaft.feedSpeed = geometry.arm * shaft.omega;
    ports[id].feedSpeed = shaft.feedSpeed;
  }
  for (const span of strands) loadLimb(span, chain, shafts);
  const state = {time: 0, ticks: 0, h, shafts, chain, strands};

  function rewind(id = 'left') {
    const shaft = shafts[id];
    if (shaft?.p.kind !== 'motor') throw new TypeError('Rewind requires a motor port');
    shaft.tripped = false;
    shaft.remaining = shaft.p.capacity;
  }

  function localLaw(shaft, metric, chainTorque) {
    const mass = chain?.linkMass ?? 0;
    const inertia = shaft.inertia + mass * metric.arm ** 2;
    const drag = shaft.drag + (chain?.linkDrag ?? 0) * metric.arm ** 2
      + (shaft.tripped ? shaft.p.brakeDrag : 0);
    const bias = mass * metric.arm * metric.slope * shaft.omega ** 2 - chainTorque - motorTorque(shaft);
    return shaftLaw(shaft, h, inertia, drag, bias);
  }

  // Exact isolated-shaft companion for a trial SI voltage. Pure until step().
  // Its slope is dI/dV on the current slip/stick branch.
  function companion(voltage) {
    if (chain) throw new TypeError('Companion is for an isolated shaft');
    if (!Number.isFinite(voltage)) throw new TypeError('Voltage must be finite');
    const shaft = shafts.left;
    const {radius} = shaft.p;
    const law = localLaw(shaft, {arm: 0, slope: 0}, 0);
    const omega = speedAt(law, voltage * radius);
    const current = radius * omega;
    const stuck = Math.abs(voltage * radius - law.offset - law.hysteresis) <= law.friction && law.friction > 0;
    const springSlope = omega >= law.lo && omega <= law.hi ? law.khStep : 0;
    const conductance = stuck ? 0 : radius ** 2 / (law.base + springSlope);
    return {current, conductance, history: current - conductance * voltage};
  }

  function tick(commands) {
    if (commands === null || typeof commands !== 'object' || Array.isArray(commands)) throw new TypeError('Drive must be an object');
    for (const [id, cmd] of Object.entries(commands)) {
      if (!Object.hasOwn(shafts, id) || !cmd || Object.keys(cmd).length !== 1 ||
        !(Object.hasOwn(cmd, 'speed') && Number.isFinite(cmd.speed) || Object.hasOwn(cmd, 'torque') && Number.isFinite(cmd.torque))) {
        throw new TypeError(`Invalid drive on ${id}`);
      }
    }
    const differential = chain ? strands[0].tension[0] - strands[1].tension[chain.links - 1] : 0;
    const opposing = chain ? strands[1].tension[0] - strands[0].tension[chain.links - 1] : 0;
    for (const [id, shaft] of entries) {
      const {p} = shaft;
      const metric = chain ? chord(shaft, chain) : {arm: 0, slope: 0};
      const chainTorque = metric.arm * (id === 'left' ? differential : opposing);
      const powerTorque = motorTorque(shaft);
      const law = localLaw(shaft, metric, chainTorque);
      const cmd = commands[id] ?? {};
      const torque = cmd.speed === undefined ? cmd.torque ?? 0 : torqueAt(law, cmd.speed);
      const omega = cmd.speed === undefined ? speedAt(law, torque) : cmd.speed;
      const advance = h * omega;
      shaft.omega = omega;
      shaft.angle += advance;
      shaft.hysteresis = clamp(shaft.hysteresis + law.khStep * omega, -law.limit, law.limit);
      shaft.chainTorque = chainTorque;
      shaft.flywheelAngle = shaft.gear * shaft.angle;
      if (p.kind === 'motor') {
        if (powerTorque) shaft.remaining = clamp(shaft.remaining - advance, 0, p.capacity);
        shaft.tripped ||= omega >= p.tripSpeed;
      }
      if (chain) {
        const next = chord(shaft, chain);
        shaft.feed = next.travel - initialTravel[id];
        shaft.feedSpeed = next.arm * omega;
      }
      const port = ports[id];
      port.angle = shaft.angle;
      port.speed = omega;
      port.torque = torque;
      port.current = p.radius * omega;
      port.voltage = torque / p.radius;
      port.chainTorque = chainTorque;
      if (chain) port.feedSpeed = shaft.feedSpeed;
      if (!Number.isFinite(shaft.angle) || !Number.isFinite(torque)) throw new RangeError(`Nonfinite shaft state at t=${state.time}`);
    }
    for (const span of strands) {
      advanceLimb(span, chain, h);
      loadLimb(span, chain, shafts);
    }
    state.time = ++state.ticks * h;
  }

  function step(dt, drive = {}) {
    numeric('dt', dt, 0, true);
    const count = Math.round(dt / h);
    if (!Number.isSafeInteger(count) || count < 1 || Math.abs(dt / h - count) > 1e-8) throw new RangeError('dt must be a positive multiple of h');
    for (let i = 0; i < count; i++) tick(typeof drive === 'function' ? drive((state.ticks + 1) * h, ports) : drive);
    return ports;
  }
  return {state, ports, step, rewind, companion};
}
