import assert from 'node:assert/strict';
import {writeFileSync} from 'node:fs';
import {create, defaults as p, STEP as h} from './model.js';

const gateTorque = V => ({torque: V * p.gateRadius});
const gateSpeed = I => ({speed: I / p.gateRadius});
const channelTorque = V => ({torque: V * p.channelRadius});
const channelSpeed = I => ({speed: I / p.channelRadius});
const torqueDrive = (gate, path) => ({gate: gateTorque(gate), channel: channelTorque(path)});
const drive = (gate, channel) => ({gate, channel});
const round = (value, digits = 5) => Number(value.toPrecision(digits));
const frequencies = [0.5, 1, 2, 4, 8, 16, 24, 30, 48];
const officialH = 1 / 30; // spintronicssimulator.js:572
const offReferenceI = 0.020; // m/s, prototype test at 2 spin mA

function run(model, seconds, input) {
  for (let n = 0; n < Math.round(seconds / h); n++) model.step(h, input);
  return model.ports;
}

function springAt(Vg, Vds = 0) {
  const model = create();
  run(model, 1.6, torqueDrive(Vg, Vds));
  return model;
}

function governorAt(Ib, Vds) {
  const model = create({mechanism: 'governor'});
  run(model, 0.8, drive(gateSpeed(Ib), channelTorque(Vds)));
  return model;
}

function fitLine(samples) {
  const avgX = samples.reduce((sum, [x]) => sum + x, 0) / samples.length;
  const avgY = samples.reduce((sum, [, y]) => sum + y, 0) / samples.length;
  const slope = samples.reduce((sum, [x, y]) => sum + (x - avgX) * (y - avgY), 0) /
    samples.reduce((sum, [x]) => sum + (x - avgX) ** 2, 0);
  const intercept = avgY - slope * avgX;
  const rms = Math.sqrt(samples.reduce((sum, [x, y]) => sum + (y - intercept - slope * x) ** 2, 0) / samples.length);
  return {slope, intercept, rms};
}

function officialDamping(opening) {
  return Math.max(0.5, 100 ** (2 * (1 - opening)) - 1); // transistor-part.js:283–301
}

function officialRaw(opening) {
  const d = officialDamping(opening);
  const J = Math.PI * (d / 100) * p.channelRadius ** 4 / 2;
  const L = J / ((1 + officialH * d) * p.channelRadius ** 2);
  return {d, L, R: L * d};
}

// Compare shapes at a shared reference point. Planck's mass/torque scale is unverified.
const off = springAt(0);
const offEffort = run(off, 0.35, drive(gateTorque(0), channelSpeed(offReferenceI))).channel.V;
const officialScale = offEffort / offReferenceI / officialRaw(0).R;
const idealScale = offEffort / offReferenceI / officialDamping(0);
const idealR = opening => idealScale * officialDamping(opening);
const officialR = opening => officialScale * officialRaw(opening).R;

function breakaway(model, gate) {
  let low = 0;
  let high = 2 * model.state.channel.staticLimit / p.channelRadius;
  if (high === 0) return 0;
  assert.equal(model.ports.channel.I, 0);
  for (let n = 0; n < 45; n++) {
    const candidate = (low + high) / 2;
    const {ports} = model.preview(h, drive(gate, channelTorque(candidate)));
    if (ports.channel.I === 0) low = candidate;
    else high = candidate;
  }
  return low;
}

const dc = [];
for (const bias of [0, 0.25, 0.5, 0.75, 1]) {
  const Vg = bias * p.fullGateEffort;
  const model = springAt(Vg);
  const gate = gateTorque(Vg);
  const opening = model.state.gate.opening;
  const staticV = breakaway(model, gate);
  const speeds = [0.02, 0.04, 0.08, 0.16, 0.32];
  const speedSweep = speeds.map(I => [I, run(model, 0.05, drive(gate, channelSpeed(I))).channel.V]);
  const sliding = fitLine(speedSweep);
  const VdsValues = Array.from({length:101}, (_, n) => (n - 50) * 0.04);
  const points = VdsValues.map(Vds => {
    const fresh = springAt(Vg);
    const I = run(fresh, 0.35, drive(gate, channelTorque(Vds))).channel.I;
    const officialLinearI = Vds / officialR(opening);
    const rotationCapI = p.channelRadius * Math.PI / (2 * officialH);
    return {Vds, I, officialLinearI, officialBelowRotationCap: Math.abs(officialLinearI) <= rotationCapI,
      idealI: Vds / idealR(opening)};
  });
  const ascending = springAt(Vg);
  const descending = springAt(Vg, VdsValues.at(-1));
  const ramp = (model, values) => values.map(Vds => [Vds, run(model, 0.35, drive(gate, channelTorque(Vds))).channel.I]);
  dc.push({Vg, opening, gateDisplacement: p.gateRadius * springAt(Vg).state.gate.angle, staticV, sliding, speedSweep,
    officialDamping: officialDamping(opening), officialR: officialR(opening), idealR: idealR(opening),
    points, ascending: ramp(ascending, VdsValues), descending: ramp(descending, [...VdsValues].reverse())});
}

const gateFit = fitLine(dc.map(row => [row.Vg, row.gateDisplacement]));
const conductanceBias = {Vg: 0.05625, Vds: 1.10, deltaVg: 0.00025};
function central(sample, x, delta) {
  return (sample(x + delta) - sample(x - delta)) / (2 * delta);
}
const {Vg, Vds, deltaVg} = conductanceBias;
const gm = {
  ...conductanceBias,
  sliding: central(V => springAt(V, Vds).ports.channel.I, Vg, deltaVg),
  stuck: central(V => springAt(V, 0.4).ports.channel.I, Vg, deltaVg),
  stuckVds: 0.4,
  official: central(V => Vds / officialR(V / p.fullGateEffort), Vg, deltaVg),
  ideal: central(V => Vds / idealR(V / p.fullGateEffort), Vg, deltaVg),
};

function captureStep(model, input, duration) {
  function sample(time) {
    const {gate, channel} = model.ports;
    return {time, gateV: gate.V, gateI: gate.I, pathV: channel.V, pathI: channel.I,
      opening: model.state.gate.opening, normal: model.state.normalForce, stuck: model.state.channel.stuck};
  }
  const trace = [sample(0)];
  for (let n = 1; n <= Math.round(duration / h); n++) {
    model.step(h, input);
    trace.push(sample(n * h));
  }
  const firstMotion = trace.find(point => point.pathI !== 0);
  const finalI = trace.at(-1).pathI;
  const initialI = trace[0].pathI;
  const target = initialI + 0.6321205588 * (finalI - initialI);
  const t63 = trace.find(point => (finalI >= initialI ? point.pathI >= target : point.pathI <= target)).time;
  return {firstMotion: firstMotion?.time ?? null, t63, initialI, finalI, trace};
}

const pathStep = captureStep(springAt(0.075), torqueDrive(0.075, 0.95), 0.25);
const gateStep = captureStep(springAt(0, 0.85), torqueDrive(0.1125, 0.85), 0.6);
const governorStep = captureStep(governorAt(0, 0.8), drive(gateSpeed(1.2), channelTorque(0.8)), 0.6);
governorStep.stopTime = governorStep.trace.find(point => point.time > 0 && point.stuck)?.time ?? null;

const governorBias = {Ib: 0.8, Vds: 1.20, deltaIb: 0.001};
const beta = {...governorBias,
  differential: central(I => governorAt(I, governorBias.Vds).ports.channel.I, governorBias.Ib, governorBias.deltaIb),
  ratio: governorAt(governorBias.Ib, governorBias.Vds).ports.channel.I / governorBias.Ib,
  sweep: Array.from({length:31}, (_, n) => {
    const Ib = n * 0.05;
    const model = governorAt(Ib, governorBias.Vds);
    return {Ib, Vg: model.ports.gate.V, Id: model.ports.channel.I, opening: model.state.gate.opening,
      beta: central(I => governorAt(I, governorBias.Vds).ports.channel.I, Ib, governorBias.deltaIb)};
  }),
};
const governorOff = governorAt(0, governorBias.Vds);
const governorR = fitLine([0.2, 0.4, 0.6, 0.8].map(Ib => {
  return [Ib, governorAt(Ib, governorBias.Vds).ports.gate.V];
})).slope;
const governorEngagesAt = Math.sqrt(p.governorPreload / (2 * p.flyballMass * p.flyballRadius)) * p.gateRadius;
const lead = p.padTravel / p.flyballStroke;
const springForce = p.governorPreload + (p.governorStiffness + p.padStiffness * lead ** 2) * p.flyballStroke;
const governorFullAt = Math.sqrt(springForce / (2 * p.flyballMass * (p.flyballRadius + p.flyballStroke))) * p.gateRadius;
beta.engagesAt = governorEngagesAt;
beta.fullAt = governorFullAt;

function complexDivide(a, b) {
  const div = b.re ** 2 + b.im ** 2;
  assert.ok(div > 0);
  return {re:(a.re * b.re + a.im * b.im) / div, im:(a.im * b.re - a.re * b.im) / div};
}

function phasor(value) {
  return {...value, magnitude: Math.hypot(value.re, value.im), phaseDeg: Math.atan2(value.im, value.re) * 180 / Math.PI};
}

function harmonic(model, hz, excitation, output, input) {
  const period = Math.round(1 / (hz * h));
  assert.ok(Math.abs(period * h * hz - 1) < 1e-12);
  const settle = Math.max(3, Math.ceil(hz));
  const measured = 5;
  const a = {re: 0, im: 0};
  const b = {re: 0, im: 0};
  for (let n = 1; n <= period * (settle + measured); n++) {
    const phase = 2 * Math.PI * (n % period) / period;
    const sin = Math.sin(phase);
    const cos = Math.cos(phase);
    const ports = model.step(h, excitation(sin));
    if (n <= period * settle) continue;
    const y = output(ports);
    const x = input(ports);
    a.re += y * sin;
    a.im += y * cos;
    b.re += x * sin;
    b.im += x * cos;
  }
  return phasor(complexDivide(a, b));
}

const dft = frequencies.map(hz => {
  const halfOpen = p.fullGateEffort / 2;
  const path = harmonic(springAt(halfOpen), hz,
    sin => drive(gateTorque(halfOpen), channelSpeed(0.24 + 0.01 * sin)), x => x.channel.V, x => x.channel.I);
  const gate = harmonic(springAt(halfOpen), hz,
    sin => drive(gateSpeed(2 * Math.PI * hz * 0.0004 * sin), channelTorque(0)), x => x.gate.V, x => x.gate.I);
  const gmAC = harmonic(springAt(Vg, Vds), hz,
    sin => torqueDrive(Vg + 0.001 * sin, Vds), x => x.channel.I, x => x.gate.V);
  const governorGate = harmonic(governorAt(governorBias.Ib, governorBias.Vds), hz,
    sin => drive(gateSpeed(governorBias.Ib + 0.005 * sin), channelTorque(governorBias.Vds)), x => x.gate.V, x => x.gate.I);
  const betaAC = harmonic(governorAt(governorBias.Ib, governorBias.Vds), hz,
    sin => drive(gateSpeed(governorBias.Ib + 0.005 * sin), channelTorque(governorBias.Vds)), x => x.channel.I, x => x.gate.I);
  const w = 2 * Math.PI * hz;
  const official = officialRaw(0.5);
  const phase = w * officialH;
  return {hz, path, gate, gm: gmAC, governorGate, beta: betaAC,
    idealPath: phasor({re: idealR(0.5), im: 0}),
    idealGate: phasor({re: 0, im: -1 / (w * gateFit.slope)}),
    officialLinearPath: hz >= 1 / (2 * officialH) ? null : phasor({
      re: officialScale * (official.R + official.L * (1 - Math.cos(phase)) / officialH),
      im: officialScale * official.L * Math.sin(phase) / officialH,
    })};
});

function fitImpedance(key, compliance = null) {
  const samples = dft.map(({hz, [key]: z}) => {
    const phase = 2 * Math.PI * hz * h;
    const d = {re: (1 - Math.cos(phase)) / h, im: Math.sin(phase) / h};
    const spring = compliance === null ? {re: 0, im: 0} : complexDivide({re: 1 / compliance, im: 0}, d);
    return {d, adjusted: {re: z.re - spring.re, im: z.im - spring.im}};
  });
  const L = samples.reduce((sum, {d, adjusted}) => sum + d.im * adjusted.im, 0) /
    samples.reduce((sum, {d}) => sum + d.im ** 2, 0);
  const R = samples.reduce((sum, {d, adjusted}) => sum + adjusted.re - L * d.re, 0) / samples.length;
  const rms = Math.sqrt(samples.reduce((sum, {d, adjusted}) => {
    return sum + (adjusted.re - R - L * d.re) ** 2 + (adjusted.im - L * d.im) ** 2;
  }, 0) / samples.length);
  return {R, L, rms, ...(compliance === null ? {} : {C: compliance})};
}

const fits = {path: fitImpedance('path'), gate: fitImpedance('gate', gateFit.slope), governorDC: {R: governorR}};
const comparison = [
  {quantity:'Gate C [m/N]', model:fits.gate.C, ideal:gateFit.slope, official:'scale unverified'},
  {quantity:'Gate series R [N s/m]', model:fits.gate.R, ideal:0, official:'scale unverified'},
  {quantity:'Gate series L [kg]', model:fits.gate.L, ideal:0, official:'scale unverified'},
  {quantity:'Path incremental R, half open [N s/m]', model:fits.path.R, ideal:idealR(0.5), official:officialR(0.5)},
  {quantity:'Path L, half open [kg]', model:fits.path.L, ideal:0, official:officialRaw(0.5).L * officialScale},
  {quantity:'Path breakaway, gate closed [N]', model:dc[0].staticV, ideal:0, official:0},
  {quantity:'Path kinetic offset, gate closed [N]', model:dc[0].sliding.intercept, ideal:0, official:0},
  {quantity:'Sliding gm at bias [(m/s)/N]', model:gm.sliding, ideal:gm.ideal, official:gm.official},
  {quantity:'Governor input DC R [N s/m]', model:governorR, ideal:'—', official:'—'},
  {quantity:'Governor incremental beta [1]', model:beta.differential, ideal:'—', official:'—'},
];

const close = (actual, expected, tolerance) => assert.ok(Math.abs(actual - expected) < tolerance,
  `Expected ${actual} within ${tolerance} of ${expected}`);
close(fits.path.R, p.channelDrag / p.channelRadius ** 2, 1e-9);
close(fits.path.L, p.channelInertia / p.channelRadius ** 2, 1e-9);
close(fits.gate.C, p.gateRadius * p.gateTravel / p.fullGateEffort, 1e-9);
close(fits.gate.R, p.gateDrag / p.gateRadius ** 2, 1e-9);
close(fits.gate.L, p.gateInertia / p.gateRadius ** 2, 1e-9);
assert.ok(dc[0].points.find(row => row.Vds === 0.80).I === 0);
assert.ok(dc[0].points.find(row => row.Vds === 1.60).I > 0);
assert.equal(dc[0].ascending.find(([V]) => V === 1.32)[1], 0);
assert.ok(dc[0].descending.find(([V]) => V === 1.32)[1] > 0);
for (const row of dc[0].points) {
  const opposite = dc[0].points.find(point => Math.abs(point.Vds + row.Vds) < 1e-12);
  close(row.I, -opposite.I, 1e-12);
}
assert.ok(gateStep.firstMotion > 0 && gateStep.finalI > 0);
assert.ok(governorStep.stopTime > 0 && governorStep.finalI === 0 && beta.differential < 0);
assert.equal(governorOff.state.gate.opening, 1);
const reverse = create({gateDirection:-1});
run(reverse, 1.6, torqueDrive(-0.075, 0));
close(reverse.state.gate.opening, 0.5, 1e-9);
const limitModel = springAt(-0.01);
const before = limitModel.state;
assert.throws(() => limitModel.step(h, drive(gateSpeed(-0.01), channelTorque(0))), /hard stop/);
assert.equal(limitModel.state, before);
const speedForced = springAt(0);
speedForced.step(h, drive(gateTorque(0), channelSpeed(0.02)));
speedForced.step(h, torqueDrive(0, -2));
assert.ok(speedForced.ports.channel.I < 0);
const governorTorque = create({mechanism:'governor'});
const governorTargetI = 0.8;
const governorInputTorque = p.governorDrag * governorTargetI / p.gateRadius;
run(governorTorque, 2, drive({torque:governorInputTorque}, channelTorque(1.2)));
close(governorTorque.ports.gate.I, governorTargetI, 1e-6);
close(governorTorque.state.gate.opening, governorAt(governorTargetI, 1.2).state.gate.opening, 1e-6);

const results = {
  units:{V:'N', I:'m/s', R:'N s/m', C:'m/N', L:'kg', time:'s', frequency:'Hz'},
  provenance:{simulationOnly:true, timeStep:h, parameters:p, offReferenceI, offEffort, officialScale, idealScale,
    officialScaleMeaning:'One-point path normalization at zero gate and 0.02 m/s, not a kit scale.',
    ideal:'Codex channel resistance law with matched off resistance, and an ideal C with the fitted travel.'},
  fits, comparison, gm, beta, dc, steps:{path: pathStep, gate: gateStep, governor: governorStep}, frequency:dft,
  checks:'PASS: fitted RLC, stiction and hysteresis, signed symmetry, gate polarity, hard stop atomicity, reversal, governor torque and speed drives',
};
writeFileSync(new URL('./results.json', import.meta.url), JSON.stringify(results, null, 2) + '\n');

console.table(comparison.map(row => Object.fromEntries(Object.entries(row).map(([key, value]) =>
  [key, typeof value === 'number' ? round(value) : value]))));
console.table(dc.map(row => ({
  'gate N': round(row.Vg), 'start N': round(row.staticV), 'slide offset N': round(row.sliding.intercept),
  'I at 1.2 N m/s': round(row.points.find(point => point.Vds === 1.2).I),
  'official d 1/s': round(row.officialDamping), 'official R N s/m': round(row.officialR),
})));
console.table(dft.map(row => ({
  Hz: row.hz, '|Z path|':round(row.path.magnitude), 'path deg':round(row.path.phaseDeg),
  '|Z gate|':round(row.gate.magnitude), 'gate deg':round(row.gate.phaseDeg),
  '|gm|':round(row.gm.magnitude), '|beta|':round(row.beta.magnitude), 'beta deg':round(row.beta.phaseDeg),
})));
console.log('Step times [s]', {channelT63:pathStep.t63, gateFirstMotion:gateStep.firstMotion, gateT63:gateStep.t63,
  governorStop:governorStep.stopTime});
console.log('Governor [m/s]', {engagesAt:governorEngagesAt, fullAt:governorFullAt, bias:governorBias.Ib,
  ratio:beta.ratio, beta:beta.differential});
console.log(results.checks);
