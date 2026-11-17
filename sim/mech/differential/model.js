/**
 * Symmetric planetary differential. Coordinates are positive around the same axis.
 * Pitch geometry and the basis of all estimates are documented in REPORT.md.
 */
export const portNames = Object.freeze(['bottom', 'middle', 'top']);
const shafts = [...portNames, 'planet'];
const meshNames = ['sun', 'ring'];
const nPlanets = 3;

// Simulator radius in notes/how-it-works.md:79. Preserve bottom; normalize to 3:2:1.
const bottomRadius = 0.033941;
export const radii = Object.freeze({
  bottom: bottomRadius, middle: 2 * bottomRadius / 3, top: bottomRadius / 3,
});

// Planet and orbit radii and three planets from the simulator's inactive sketch
// (junction-part.js:8–9,132–138,155–157). Remaining values follow from pitch geometry.
const planetRadius = 0.003;
const orbit = 0.009;
const sunRadius = orbit - planetRadius;
const ringRadius = orbit + planetRadius;
export const geometry = Object.freeze({
  nPlanets, orbit, sunRadius, ringRadius, planetRadius,
  sunTeeth: 24, ringTeeth: 48, planetTeeth: 12,
});

export const defaults = Object.freeze({
  // Estimated disk/annulus inertias (kg m²), excluding planet orbital and spin inertia.
  carrierInertia: 0.5 * 0.010 * radii.bottom ** 2,
  ringInertia: 0.5 * 0.008 * (radii.middle ** 2 + ringRadius ** 2),
  sunInertia: 0.5 * 0.003 * radii.top ** 2,
  planetInertia: 0.5 * 0.00012 * planetRadius ** 2,
  planetMass: 0.00012, // kg, per planet, estimate
  bottomDrag: 3e-6, middleDrag: 2e-6, topDrag: 1e-6, // N m s/rad, estimated bearing drag
  meshStiffness: 2e4, // N/m, per planet per mesh, estimate
  meshDamping: 0.6, // N s/m, per planet per mesh, estimate
  backlash: 50e-6, // m, half-clearance at each pitch line, estimate
  meshFriction: 1.5e-6, // N m s/rad, sliding drag per planet per mesh at frictionLoad, estimate
  frictionLoad: 0.1, // N, per planet per mesh, estimated reference force for sliding drag
  fixedStep: 1 / 9600, // s, numerical choice
});

const positive = new Set([
  'carrierInertia', 'ringInertia', 'sunInertia', 'planetInertia',
  'planetMass', 'meshStiffness', 'frictionLoad', 'fixedStep',
]);

// dg/dq, including carrier-relative motion and both rolling directions.
const gradients = [
  [-sunRadius - planetRadius, 0, sunRadius, planetRadius],
  [-ringRadius + planetRadius, ringRadius, 0, -planetRadius],
];
const matingShaft = [2, 1];
const allModes = [[0, 0], [-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [1, 1], [-1, 1], [1, -1]];
const entries = (values) => Object.fromEntries(shafts.map((name, i) => [name, values[i]]));
const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
const zeros = () => [0, 0, 0, 0];

function parseDrive(drive) {
  if (!drive || typeof drive !== 'object' || Array.isArray(drive)) throw new TypeError('Expected a port drive object');
  for (const key of Object.keys(drive)) {
    if (!portNames.includes(key)) throw new TypeError('Unknown drive port ' + key);
  }
  const torque = zeros();
  const speed = zeros();
  const free = [3];
  for (const [i, key] of portNames.entries()) {
    if (!Object.hasOwn(drive, key)) {
      free.push(i);
      continue;
    }
    const command = drive[key];
    if (!command || typeof command !== 'object' || Object.keys(command).length !== 1 ||
        !((Object.hasOwn(command, 'torque') && Number.isFinite(command.torque)) ||
          (Object.hasOwn(command, 'speed') && Number.isFinite(command.speed)))) {
      throw new TypeError(key + ' drive requires exactly one finite speed or torque');
    }
    if (Object.hasOwn(command, 'torque')) {
      torque[i] = command.torque;
      free.push(i);
    } else {
      speed[i] = command.speed;
    }
  }
  return {torque, speed, free};
}

// Factor the free block once and reuse it for the unconstrained motion and both mesh responses.
function factor(matrix, free) {
  const l = free.map(() => new Float64Array(free.length));
  for (let i = 0; i < free.length; i++) {
    for (let j = 0; j <= i; j++) {
      let value = matrix[free[i]][free[j]];
      for (let k = 0; k < j; k++) value -= l[i][k] * l[j][k];
      if (j === i && !(value > 0 && Number.isFinite(value))) throw new Error('Invalid inertia matrix');
      l[i][j] = i === j ? Math.sqrt(value) : value / l[j][j];
    }
  }
  return l;
}

function solve(l, free, rhs) {
  const x = zeros();
  for (let i = 0; i < free.length; i++) {
    let value = rhs[free[i]];
    for (let j = 0; j < i; j++) value -= l[i][j] * x[free[j]];
    x[free[i]] = value / l[i][i];
  }
  for (let i = free.length - 1; i >= 0; i--) {
    let value = x[free[i]];
    for (let j = i + 1; j < free.length; j++) value -= l[j][i] * x[free[j]];
    x[free[i]] = value / l[i][i];
  }
  return x;
}

// g = predicted - compliance * force. Each mesh is open or loaded on one flank.
function contact(predicted, compliance, lower, upper, stiffness) {
  const [s00, s01, s11] = compliance;
  const a = s00 + 1 / stiffness;
  const d = s11 + 1 / stiffness;
  const modeAt = (g, i) => g < lower[i] ? -1 : g > upper[i] ? 1 : 0;
  const guessed = predicted.map(modeAt);
  for (const modes of [guessed, ...allModes]) {
    const [m0, m1] = modes;
    const b0 = predicted[0] - (m0 > 0 ? upper[0] : lower[0]);
    const b1 = predicted[1] - (m1 > 0 ? upper[1] : lower[1]);
    let f0 = 0;
    let f1 = 0;
    if (m0 && m1) {
      const determinant = a * d - s01 ** 2;
      if (!(determinant > 0)) throw new Error('Invalid mesh compliance matrix');
      f0 = (d * b0 - s01 * b1) / determinant;
      f1 = (a * b1 - s01 * b0) / determinant;
    } else if (m0) {
      f0 = b0 / a;
    } else if (m1) {
      f1 = b1 / d;
    }
    if (m0 * f0 < 0 || m1 * f1 < 0) continue;
    const g0 = predicted[0] - s00 * f0 - s01 * f1;
    const g1 = predicted[1] - s01 * f0 - s11 * f1;
    if ((m0 || modeAt(g0, 0) === 0) && (m1 || modeAt(g1, 1) === 0)) return [f0, f1];
  }
  throw new Error('No consistent mesh flanks');
}

/**
 * Starts at rest, with both clearances centered. Omitted ports receive zero torque.
 * step(dt, {bottom: {speed: rad/s}, middle: {torque: N m}, ...})
 * advances an integer number of fixed steps, holding each command over dt.
 * Outputs are the final substep values. Positive torque and power enter the part.
 * The state and port records are live, read-only to callers.
 */
export function create(params = {}) {
  for (const [name, value] of Object.entries(params)) {
    if (!Object.hasOwn(defaults, name)) throw new TypeError('Unknown parameter ' + name);
    if (!Number.isFinite(value) || value < 0 || (positive.has(name) && value === 0)) {
      throw new RangeError('Invalid parameter ' + name);
    }
  }
  const p = {...defaults, ...params};
  const h = p.fixedStep;
  const inertia = [
    p.carrierInertia + nPlanets * p.planetMass * orbit ** 2,
    p.ringInertia, p.sunInertia, nPlanets * p.planetInertia,
  ];
  const bearing = [p.bottomDrag, p.middleDrag, p.topDrag, 0];
  const damping = p.meshDamping / h;
  const stiffness = p.meshStiffness + damping;
  const gap = p.backlash;
  const angle = zeros();
  const omega = zeros();
  const displacement = [0, 0];
  let steps = 0;

  const ports = Object.fromEntries(portNames.map(name => [name, {
    radius: radii[name], speed: 0, torque: 0, current: 0, voltage: 0, travel: 0,
  }]));
  const state = {
    time: 0, angle: entries(angle), omega: entries(omega),
    mesh: Object.fromEntries(meshNames.map(name => [name, {displacement: 0, force: 0, flank: 0}])),
    energy: 0, bearingLoss: 0, slidingLoss: 0, currentResidual: 0,
  };
  const meshes = meshNames.map(name => state.mesh[name]);

  function advance({torque, speed, free}) {
    const drag = meshes.map(mesh => nPlanets * p.meshFriction *
      Math.min(1, Math.abs(mesh.force) / p.frictionLoad));
    const matrix = inertia.map((mass, i) => {
      const row = zeros();
      row[i] = mass + h * bearing[i];
      return row;
    });
    for (const [mesh, i] of matingShaft.entries()) {
      const hd = h * drag[mesh];
      matrix[i][i] += hd;
      matrix[3][3] += hd;
      matrix[i][3] -= hd;
      matrix[3][i] -= hd;
    }
    const l = factor(matrix, free);
    const rhs = inertia.map((mass, i) => mass * omega[i] + h * torque[i] - dot(matrix[i], speed));
    const freeSpeed = solve(l, free, rhs).map((w, i) => w + speed[i]);
    const predicted = gradients.map((gradient, i) => displacement[i] + h * dot(gradient, freeSpeed));
    const lower = displacement.map(g => -gap - damping / stiffness * Math.max(-g - gap, 0));
    const upper = displacement.map(g => gap + damping / stiffness * Math.max(g - gap, 0));
    const response = gradients.map(gradient => solve(l, free, gradient).map(value => value * h * nPlanets));
    const compliance = [
      h * dot(gradients[0], response[0]),
      h * dot(gradients[0], response[1]),
      h * dot(gradients[1], response[1]),
    ];
    const force = contact(predicted, compliance, lower, upper, stiffness);
    const next = freeSpeed.map((w, i) => w - response[0][i] * force[0] - response[1][i] * force[1]);
    const reaction = portNames.map((_, i) => free.includes(i) ? torque[i] :
      (dot(matrix[i], next) - inertia[i] * omega[i]) / h +
      nPlanets * (gradients[0][i] * force[0] + gradients[1][i] * force[1]));
    if (![...next, ...reaction, ...force].every(Number.isFinite)) throw new Error('Non-finite junction solution');

    state.energy = 0;
    state.bearingLoss = 0;
    for (const [i, name] of shafts.entries()) {
      omega[i] = next[i];
      angle[i] += h * next[i];
      state.angle[name] = angle[i];
      state.omega[name] = omega[i];
      state.energy += 0.5 * inertia[i] * next[i] ** 2;
      state.bearingLoss += bearing[i] * next[i] ** 2;
    }
    state.slidingLoss = 0;
    for (const [i, mesh] of meshes.entries()) {
      displacement[i] += h * dot(gradients[i], next);
      mesh.displacement = displacement[i];
      mesh.force = force[i];
      mesh.flank = force[i] === 0 ? 0 : Math.sign(force[i]);
      state.energy += 0.5 * nPlanets * p.meshStiffness * Math.max(Math.abs(displacement[i]) - gap, 0) ** 2;
      state.slidingLoss += drag[i] * (next[matingShaft[i]] - next[3]) ** 2;
    }
    for (const [i, name] of portNames.entries()) {
      const port = ports[name];
      port.speed = next[i];
      port.torque = reaction[i];
      port.current = radii[name] * next[i];
      port.voltage = reaction[i] / radii[name];
      port.travel = radii[name] * angle[i];
    }
    state.currentResidual = ports.bottom.current - ports.middle.current - ports.top.current;
    state.time = ++steps * h;
  }

  function step(dt, drive = {}) {
    if (!Number.isFinite(dt)) throw new TypeError('dt must be a finite number');
    const count = Math.round(dt / h);
    if (!Number.isSafeInteger(count) || count < 1 || Math.abs(dt / h - count) > 1e-9) {
      throw new RangeError('dt must be a positive integer multiple of fixedStep');
    }
    const constraints = parseDrive(drive);
    for (let i = 0; i < count; i++) advance(constraints);
    return ports;
  }

  return {state, step, ports};
}
