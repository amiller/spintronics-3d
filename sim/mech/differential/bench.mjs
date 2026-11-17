import assert from 'node:assert/strict';
import {writeFileSync} from 'node:fs';
import {create, defaults as p, geometry as g, portNames, radii as r} from './model.js';

const h = p.fixedStep;
const ratio = g.sunRadius / r.top;
const preload = 0.25; // N per planet at each mesh
const locked = {middle: {speed: 0}, top: {speed: 0}};
const compatible = (middle, top, bottom = middle + top) => ({
  bottom: {speed: bottom / r.bottom},
  middle: {speed: middle / r.middle},
  top: {speed: top / r.top},
});
const drop = ({bottom, middle, top}) => [bottom.voltage + middle.voltage, bottom.voltage + top.voltage];
const through = ({bottom, middle, top}) => [bottom.voltage, -middle.voltage, -top.voltage];
const power = (ports) => portNames.reduce((sum, name) => sum + ports[name].voltage * ports[name].current, 0);
const isSeated = ({mesh}) => mesh.sun.flank === -1 && mesh.ring.flank === -1;
const near = (actual, expected, tolerance, label) =>
  assert.ok(Math.abs(actual - expected) <= tolerance, `${label}: got ${actual}, expected ${expected} ± ${tolerance}`);

function seat(model, step = h) {
  const distance = 2 * (p.backlash + preload / p.meshStiffness) / ratio;
  const ticks = 150;
  model.step(ticks * step, compatible(0, 0, distance / (ticks * step)));
  model.step(1000 * step, compatible(0, 0));
  assert.ok(isSeated(model.state));
  for (const mesh of Object.values(model.state.mesh)) near(mesh.force, -preload, 1e-7, 'preload');
}

function dcSweep() {
  const rows = [];
  for (const input of [0, 1]) {
    const model = create();
    seat(model);
    for (const current of [-0.24, -0.16, -0.08, 0, 0.08, 0.16, 0.24]) {
      const drive = compatible(input === 0 ? current : 0, input === 1 ? current : 0);
      model.step(1920 * h, drive);
      assert.ok(isSeated(model.state));
      const {bearingLoss, slidingLoss} = model.state;
      near(power(model.ports), bearingLoss + slidingLoss, 1e-11, 'steady power balance');
      rows.push({input, current, through: through(model.ports), drop: drop(model.ports),
        bearingLoss, slidingLoss});
    }
  }
  const resistance = [0, 1].map(output => [0, 1].map(input => {
    const column = rows.filter(row => row.input === input);
    return column.reduce((sum, row) => sum + row.current * row.drop[output], 0) /
      column.reduce((sum, row) => sum + row.current ** 2, 0);
  }));
  return {ports: ['middle', 'top'], resistance, rows};
}

function blockedStep(step = h, voltage = 0.15, model = create({fixedStep: step})) {
  const drive = {...locked, bottom: {torque: r.bottom * voltage}};
  const start = model.state.time;
  const samples = [];
  let engagedAt = null;
  let peak = 0;
  let peakTravel = 0;
  let settlingTime = 0;
  const target = Math.sign(voltage) * 2 * p.backlash / ratio +
    2 * voltage / (g.nPlanets * p.meshStiffness * ratio ** 2);
  const count = Math.round(0.6 / step);
  const stride = Math.round(0.001 / step);
  const targetFlank = -Math.sign(voltage);
  for (let i = 1; i <= count; i++) {
    model.step(step, drive);
    const {bottom} = model.ports;
    const elapsed = model.state.time - start;
    const {sun, ring} = model.state.mesh;
    if (engagedAt === null && sun.flank === targetFlank && ring.flank === targetFlank) engagedAt = elapsed;
    peak = Math.max(peak, Math.abs(bottom.current));
    peakTravel = Math.max(peakTravel, Math.abs(bottom.travel));
    if (Math.abs(bottom.travel - target) > 0.02 * Math.abs(target)) settlingTime = elapsed + step;
    const sampleStride = elapsed < 0.17 ? stride : stride * 20;
    if (i % sampleStride === 0 || i === count) samples.push({
      time: elapsed, current: bottom.current, travel: bottom.travel,
      through: through(model.ports), meshForces: [sun.force, ring.force],
    });
  }
  const final = samples.at(-1);
  assert.notEqual(engagedAt, null);
  near(final.travel, target, 1e-9, 'blocked static travel');
  final.through.forEach(value => near(value, voltage, 1e-7, 'blocked static effort'));
  return {voltage, engagedAt, peakCurrent: peak, peakTravel, settlingTime, final, samples};
}

function staticCompliance() {
  const rows = [-0.30, -0.20, -0.10, 0.10, 0.20, 0.30].map(voltage => {
    const model = create();
    model.step(5760 * h, {...locked, bottom: {torque: r.bottom * voltage}});
    return {voltage, travel: model.ports.bottom.travel};
  });
  const x = rows.map(row => Math.abs(row.voltage));
  const y = rows.map(row => Math.abs(row.travel));
  const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
  const mx = mean(x), my = mean(y);
  const compliance = x.reduce((sum, value, i) => sum + (value - mx) * (y[i] - my), 0) /
    x.reduce((sum, value) => sum + (value - mx) ** 2, 0);
  return {compliance, oneSidedPlay: my - compliance * mx, rows};
}

function reversal() {
  const model = create();
  const current = 0.002;
  seat(model);
  let opens = null;
  let closes = null;
  const samples = [];
  for (let i = 1; i <= Math.ceil(8 * p.backlash / (ratio * current * h)); i++) {
    model.step(h, compatible(0, 0, -current));
    const {sun, ring} = model.state.mesh;
    const x = model.ports.bottom.travel;
    if (opens === null && sun.flank === 0 && ring.flank === 0) opens = x;
    if (opens !== null && (sun.flank === 1 || ring.flank === 1)) {
      closes = x;
      break;
    }
    if (i % 16 === 0) samples.push({travel: x, through: through(model.ports), meshForces: [sun.force, ring.force]});
  }
  assert.notEqual(closes, null, 'opposite flank was never reached');
  return {current: -current, opens, closes, width: opens - closes, samples};
}

const frequencies = [0.5, 1, 2, 4, 8, 16, 32, 64, 96, 192];
const bias = 0.04;
const amplitude = 0.003;
function impedance(frequency, input) {
  const model = create();
  seat(model);
  model.step(2400 * h, compatible(bias, bias));
  assert.ok(isSeated(model.state));
  const period = Math.round(1 / (h * frequency));
  assert.ok(Math.abs(period * h * frequency - 1) < 1e-12);
  const skip = Math.max(3 * period, 4800);
  const samples = 8 * period;
  const re = [0, 0], im = [0, 0];
  for (let i = 1; i <= skip + samples; i++) {
    const phase = 2 * Math.PI * (i % period) / period;
    const signal = amplitude * Math.cos(phase);
    model.step(h, compatible(bias + (input === 0 ? signal : 0), bias + (input === 1 ? signal : 0)));
    assert.ok(isSeated(model.state), 'small signal crossed backlash');
    if (i <= skip) continue;
    drop(model.ports).forEach((value, output) => {
      re[output] += value * Math.cos(phase);
      im[output] -= value * Math.sin(phase);
    });
  }
  return re.map((sum, output) => {
    const real = 2 * sum / (samples * amplitude);
    const imaginary = 2 * im[output] / (samples * amplitude);
    return {real, imaginary, magnitude: Math.hypot(real, imaginary),
      phaseDegrees: Math.atan2(imaginary, real) * 180 / Math.PI};
  });
}

function frequencySweep() {
  const rows = frequencies.map(frequency => ({
    frequency, columns: [impedance(frequency, 0), impedance(frequency, 1)],
  }));
  const fit = rows.filter(row => row.frequency <= 8);
  // Z = R + j 2πf L. Fit real and imaginary parts independently.
  const resistance = [0, 1].map(output => [0, 1].map(input =>
    fit.reduce((sum, row) => sum + row.columns[input][output].real, 0) / fit.length));
  const inductance = [0, 1].map(output => [0, 1].map(input =>
    fit.reduce((sum, row) => sum + 2 * Math.PI * row.frequency * row.columns[input][output].imaginary, 0) /
    fit.reduce((sum, row) => sum + (2 * Math.PI * row.frequency) ** 2, 0)));
  return {bias, amplitude, resistance, inductance, rows};
}

function rigidLimit() {
  const transform = [[1/r.bottom, 1/r.bottom], [1/r.middle, 0], [0, 1/r.top], [1/r.top, -1/r.top]];
  const inertia = [p.carrierInertia + g.nPlanets * p.planetMass * g.orbit ** 2,
    p.ringInertia, p.sunInertia, g.nPlanets * p.planetInertia];
  const bearing = [p.bottomDrag, p.middleDrag, p.topDrag, 0];
  const slip = [2, 1].map(i => transform[i].map((value, j) => value - transform[3][j]));
  const gram = (weights, matrix, i, j) => weights.reduce((sum, weight, k) =>
    sum + weight * matrix[k][i] * matrix[k][j], 0);
  const inductance = [0, 1].map(i => [0, 1].map(j => gram(inertia, transform, i, j)));
  const resistance = [0, 1].map(i => [0, 1].map(j =>
    gram(bearing, transform, i, j) + gram([g.nPlanets * p.meshFriction, g.nPlanets * p.meshFriction], slip, i, j)));
  return {resistance, inductance, width: 4 * p.backlash / ratio,
    compliance: 2 / (g.nPlanets * p.meshStiffness * ratio ** 2)};
}

function mixedDrive() {
  const model = create();
  let maxEnergyResidual = 0;
  for (let i = 1; i <= 9600; i++) {
    const oldEnergy = model.state.energy;
    model.step(h, {
      bottom: {speed: 0.12 / r.bottom}, middle: {speed: 0.04 / r.middle},
      top: {torque: -r.top * 0.2},
    });
    const {bearingLoss, slidingLoss, energy} = model.state;
    const residual = energy - oldEnergy - h * (power(model.ports) - bearingLoss - slidingLoss);
    maxEnergyResidual = Math.max(residual, maxEnergyResidual);
    assert.ok(residual < 1e-11, 'discrete energy inequality violated');
  }
  assert.ok(isSeated(model.state));
  near(model.ports.top.current, 0.08, 1e-8, 'emergent KCL with mixed drive');
  near(model.state.omega.planet - model.state.omega.bottom,
    -g.sunRadius / g.planetRadius * (model.state.omega.top - model.state.omega.bottom), 1e-6, 'planet rolling');
  return {current: portNames.map(name => model.ports[name].current),
    through: through(model.ports), residual: model.state.currentResidual, maxEnergyResidual};
}

const dc = dcSweep();
const positiveStep = blockedStep();
const negativeStep = blockedStep(h, -0.15, (() => {
  const model = create();
  model.step(5760 * h, {...locked, bottom: {torque: r.bottom * 0.15}});
  return model;
})());
const finer = blockedStep(h / 2);
const fineTransient = blockedStep(h / 16);
const compliance = staticCompliance();
const backlash = reversal();
const ac = frequencySweep();
const rigid = rigidLimit();
const mixed = mixedDrive();

for (const output of [0, 1]) for (const input of [0, 1]) {
  near(dc.resistance[output][input], rigid.resistance[output][input], 1e-8, 'DC drag fit');
  near(ac.inductance[output][input], rigid.inductance[output][input], 0.0001, 'AC inertia fit');
}
near(compliance.compliance, rigid.compliance, 1e-9, 'compliance fit');
near(backlash.width, rigid.width, 1e-6, 'backlash measurement');
near(finer.peakCurrent, positiveStep.peakCurrent, 0.05 * finer.peakCurrent, 'half-step peak current');
near(finer.engagedAt, positiveStep.engagedAt, h, 'half-step contact arrival');
near(fineTransient.peakCurrent, positiveStep.peakCurrent, 0.05 * fineTransient.peakCurrent, 'fine-step peak current');
const validation = create();
assert.throws(() => validation.step(h / 2), /integer multiple/);
assert.throws(() => validation.step(h, {bottom: {speed: 1, torque: 2}}), /exactly one/);
assert.throws(() => create({meshStiffness: 0}), /Invalid parameter/);
assert.equal(validation.state.time, 0);

const pairName = (output, input) => `${['middle', 'top'][output]} ← ${['middle', 'top'][input]}`;
const n = value => value.toPrecision(6);
const pairs = [0, 1].flatMap(i => [0, 1].map(j => [i, j]));
console.table([
  ...pairs.map(([i, j]) => ({metric: 'DC R ' + pairName(i, j) + ' [N s/m]', model: n(dc.resistance[i][j]), ideal: '0'})),
  ...pairs.map(([i, j]) => ({metric: 'AC L ' + pairName(i, j) + ' [kg]', model: n(ac.inductance[i][j]), ideal: '0'})),
  {metric: 'Blocked compliance [µm/N]', model: n(compliance.compliance * 1e6), ideal: '0'},
  {metric: 'Reversal dead band [µm]', model: n(backlash.width * 1e6), ideal: '0'},
  {metric: 'Blocked step peak |I| [m/s]', model: n(positiveStep.peakCurrent), ideal: '0'},
  {metric: 'Blocked step mesh engagement [ms]', model: n(positiveStep.engagedAt * 1000), ideal: '0'},
  {metric: 'Blocked step 2% settling [ms]', model: n(positiveStep.settlingTime * 1000), ideal: '0'},
  {metric: 'Mixed-drive KCL residual [m/s]', model: n(mixed.residual), ideal: '0'},
]);
console.table(ac.rows.map(({frequency, columns}) => ({
  'Hz': frequency,
  '|Zmm| [N s/m]': n(columns[0][0].magnitude), '∠Zmm [°]': n(columns[0][0].phaseDegrees),
  '|Ztt| [N s/m]': n(columns[1][1].magnitude), '∠Ztt [°]': n(columns[1][1].phaseDegrees),
})));
console.log('PASS: DC drag, AC inertia, blocked compliance, backlash, timestep refinement, mixed-drive KCL, energy and API checks.');

const results = {
  units: {current: 'm/s', voltage: 'N', travel: 'm', time: 's', resistance: 'N s/m',
    inductance: 'kg', compliance: 'm/N', loss: 'W', meshForce: 'N per planet'},
  parameters: p, radii: r, geometry: g,
  ideal: {currentResidual: 0, effortDrop: 0, resistance: 0, inductance: 0, compliance: 0,
    reversalWidth: 0, blockedStepCurrent: 0, blockedStepEngagementTime: 0},
  rigidPrediction: rigid, dc, step: {positive: positiveStep, reversal: negativeStep,
    refinement: [[h / 2, finer], [h / 16, fineTransient]].map(([dt, step]) => ({
      dt, peakCurrent: step.peakCurrent, engagedAt: step.engagedAt, settlingTime: step.settlingTime,
    }))},
  compliance, backlash, ac, mixed,
};
writeFileSync(new URL('./results.json', import.meta.url), JSON.stringify(results, null, 2) + '\n');
