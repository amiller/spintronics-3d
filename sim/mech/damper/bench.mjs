import assert from 'node:assert/strict';
import { writeFileSync } from 'node:fs';
import { create, KIT_LABELS, SIM_LABELS, SCIENCE_MAX_LABEL, PITCH_RADIUS as radius, SPIN_SCALE as scale } from './model.js';

const dcDt = 1 / 4800;
const acSampleRate = 4800;
const acDt = 1 / acSampleRate;
const stepDt = 1 / 768000;
const stepSpinV = 3;
const voltages = [-6, -5, -4, -3, -2, -1, -0.5, -0.25, 0, 0.25, 0.5, 1, 2, 3, 4, 5, 6];
const frequencies = [0.5, 1, 2, 5, 10, 20, 50, 100];
const torqueFor = voltage => voltage * radius / scale;
const spinCurrent = port => port.current / scale;
const spinVoltage = port => port.voltage * scale;

function near(actual, expected, tolerance, context) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${context}: ${actual} vs ${expected} (tolerance ${tolerance})`);
}

function dcPoint(params, spinV) {
  const model = create(params);
  const torque = torqueFor(spinV);
  const drive = { top: { torque } };
  for (let sample = 1; sample <= 10 / dcDt; sample++) {
    const port = model.step(dcDt, drive).top;
    if (Math.abs(model.state.dragTorque - torque) <= 2e-9 * Math.abs(torque) + 1e-15) {
      return { spinV, spinA: spinCurrent(port), timeSeconds: sample * dcDt };
    }
  }
  throw new Error(`DC point did not settle at ${spinV} spin V: ${JSON.stringify(params)}`);
}

function dcSweep(params, idealR) {
  const points = voltages.map(voltage => {
    const point = dcPoint(params, voltage);
    const idealSpinA = voltage / idealR;
    return { ...point, idealSpinA, secantSpinOhm: voltage === 0 ? null : voltage / point.spinA,
      currentErrorPct: voltage === 0 ? null : 100 * (point.spinA / idealSpinA - 1) };
  });
  const fitSpinOhm = points.reduce((sum, p) => sum + p.spinA * p.spinV, 0) /
    points.reduce((sum, p) => sum + p.spinA ** 2, 0);
  const maxCurrentErrorPct = Math.max(...points.filter(p => p.spinV !== 0).map(p => Math.abs(p.currentErrorPct)));
  const lineFitRmsSpinV = Math.sqrt(points.reduce((sum, p) => sum + (p.spinV - fitSpinOhm * p.spinA) ** 2, 0) / points.length);
  const probes = [-1e-4, 1e-4].map(voltage => dcPoint(params, voltage));
  assert.ok(probes[0].spinA < 0 && probes[1].spinA > 0, 'No simulated deadband at probe voltages');
  for (let i = 0; i < points.length; i++) {
    near(points[i].spinA, -points.at(-1 - i).spinA, 1e-12, 'Odd DC curve');
  }
  return { points, fitSpinOhm, lineFitRmsSpinV, maxCurrentErrorPct, nearZeroProbes: probes };
}

function torqueStep(params, settled, dt, keepTrace = true) {
  const model = create(params);
  const drive = { top: { torque: torqueFor(stepSpinV) } };
  const endSpeed = settled.spinA * scale / radius;
  const slope = (model.steady(endSpeed * 1.00001) - model.steady(endSpeed * 0.99999)) / (2e-5 * endSpeed);
  const localTau = model.properties.inertia / slope;
  const samples = Math.ceil(14 * localTau / dt);
  const stride = Math.max(1, Math.floor(samples / 120));
  const targets = [0.1, 1 - Math.exp(-1), 0.9];
  const crossings = [];
  const trace = [{ timeSeconds: 0, spinV: stepSpinV, spinA: 0 }];
  let prior = 0;
  for (let k = 1; k <= samples; k++) {
    const port = model.step(dt, drive).top;
    const value = spinCurrent(port);
    if (crossings.length < targets.length) {
      const target = targets[crossings.length] * settled.spinA;
      if (prior < target && value >= target) crossings.push((k - 1 + (target - prior) / (value - prior)) * dt);
    }
    if (keepTrace && (k % stride === 0 || k === samples)) {
      trace.push({ timeSeconds: k * dt, spinV: spinVoltage(port), spinA: value });
    }
    assert.ok(value >= prior, 'Positive torque step has monotonic speed');
    prior = value;
  }
  assert.equal(crossings.length, 3, 'Step must reach all crossing thresholds');
  near(prior, settled.spinA, settled.spinA * 2e-6, 'Settled transient');
  return { spinV: stepSpinV, finalSpinA: settled.spinA, t10Seconds: crossings[0],
    t63Seconds: crossings[1], t90Seconds: crossings[2], localTauSeconds: localTau, trace };
}

function impedance(params, bias, hz, idealR) {
  const model = create(params);
  const amplitude = 0.01 * (bias.spinA === 0 ? stepSpinV / idealR : Math.abs(bias.spinA));
  const perCycle = acSampleRate / hz;
  assert.ok(Number.isInteger(perCycle), 'Coherent sampling requires an integral sample count');
  const phaseStep = 2 * Math.PI / perCycle;
  const drive = { top: { speed: bias.spinA * scale / radius } };
  model.step(acDt, drive);
  let realV = 0, imagV = 0, realI = 0, imagI = 0;
  for (let k = 1; k <= 2 * perCycle; k++) {
    const phase = k * phaseStep;
    const cos = Math.cos(phase);
    const sin = Math.sin(phase);
    drive.top.speed = (bias.spinA + amplitude * cos) * scale / radius;
    const port = model.step(acDt, drive).top;
    if (k <= perCycle) continue;
    const voltage = spinVoltage(port) - bias.spinV;
    const current = spinCurrent(port) - bias.spinA;
    realV += voltage * cos;
    imagV -= voltage * sin;
    realI += current * cos;
    imagI -= current * sin;
  }
  const denominator = realI ** 2 + imagI ** 2;
  const realSpinOhm = (realV * realI + imagV * imagI) / denominator;
  const imagSpinOhm = (imagV * realI - realV * imagI) / denominator;
  return { hz, amplitudeSpinA: amplitude, realSpinOhm, imagSpinOhm,
    magnitudeSpinOhm: Math.hypot(realSpinOhm, imagSpinOhm), phaseDeg: Math.atan2(imagSpinOhm, realSpinOhm) * 180 / Math.PI,
    idealMagnitudeSpinOhm: idealR, idealPhaseDeg: 0 };
}

function acSweep(params, bias, idealR) {
  const points = frequencies.map(hz => impedance(params, bias, hz, idealR));
  const basis = points.map(({ hz }) => Math.sin(2 * Math.PI * hz * acDt) / acDt);
  const fitSpinH = points.reduce((sum, p, i) => sum + p.imagSpinOhm * basis[i], 0) /
    basis.reduce((sum, value) => sum + value ** 2, 0);
  const fitSpinOhm = points.reduce((sum, p) =>
    sum + p.realSpinOhm - fitSpinH * (1 - Math.cos(2 * Math.PI * p.hz * acDt)) / acDt, 0) / points.length;
  const expectedSpinH = create(params).properties.inductance * scale ** 2;
  near(fitSpinH, expectedSpinH, 1e-8, 'AC fit recovers rotor inertia');
  return { biasSpinV: bias.spinV, biasSpinA: bias.spinA, points, fitSpinOhm, fitSpinH };
}

function characterize(params, idealR, source) {
  const { properties } = create(params);
  const dc = dcSweep(params, idealR);
  const settled = dc.points.find(p => p.spinV === stepSpinV);
  const step = torqueStep(params, settled, stepDt);
  const fineStep = torqueStep(params, settled, stepDt / 2, false);
  const stepT63RefinementPct = 100 * Math.abs(step.t63Seconds / fineStep.t63Seconds - 1);
  assert.ok(stepT63RefinementPct < 0.5, 'Time-step refinement changes t63 by less than 0.5%');
  const ac = [dc.points.find(p => p.spinV === 0), settled].map(bias => acSweep(params, bias, idealR));
  if (params.kind === 'oil') near(settled.secantSpinOhm, idealR, 1e-5, 'Oil calibration at 3 spin V');
  else near(step.t63Seconds, step.localTauSeconds, stepDt, 'Eddy step agrees with R–L time constant');
  return { source, idealSpinOhm: idealR, properties, dc, step, ac, stepT63RefinementPct };
}

function checkPorts() {
  for (const kind of ['oil', 'eddy']) {
    const { step, reaction, ports, state, properties } = create({ kind });
    const h = 0.05;
    const rate = reaction(h, -10);
    step(h, { bottom: { speed: -10 }, middle: { torque: 0.001 } });
    near(ports.bottom.torque + ports.middle.torque, rate.torque, 1e-15, 'Shared-shaft torque balance');
    near(ports.middle.current, ports.bottom.current, 1e-15, 'Shared-shaft chain speed');
    assert.equal(ports.top.voltage, 0, 'Unloaded sprocket tension difference');
    assert.throws(() => step(h, { top: { speed: 1 }, bottom: { speed: 1 } }), /only one imposed speed/);
    assert.throws(() => step(h, { top: { torque: NaN } }), /finite/);
    assert.equal(state.omega, -10, 'Rejected drive leaves the previous state intact');
    const energyBefore = properties.inertia * state.omega ** 2 / 2;
    step(h);
    const energyAfter = properties.inertia * state.omega ** 2 / 2;
    assert.ok(state.omega < 0 && energyAfter < energyBefore, 'Passive coast with a large step');
    const before = reaction(h, 15);
    const after = reaction(h, 15.00001);
    near((after.voltage - before.voltage) / (0.00001 * radius), before.resistance, 1e-6, 'MNA reaction slope');
  }
}

checkPorts();
const cases = SIM_LABELS.map(label => characterize({ kind: 'oil', label }, label,
  KIT_LABELS.includes(label) ? 'Upper Story kit label' : 'official simulator label; hardware unverified'));
cases.push(characterize({ kind: 'oil', label: SCIENCE_MAX_LABEL }, SCIENCE_MAX_LABEL,
  'Upper Story science guide range endpoint; kit inclusion unverified'));
cases.push(characterize({ kind: 'eddy' }, 50, 'design estimate compared with 50 spin Ω'));

const results = {
  basis: 'Simulation of estimated mechanisms. Resistance and inductance are in spin Ω and spin H.',
  portSI: { pitchRadiusMetres: radius, metresPerSpinCoulomb: scale },
  protocol: { dcDtSeconds: dcDt, stepDtSeconds: stepDt, acDtSeconds: acDt,
    stepSpinV, frequenciesHz: frequencies, dcSweepSpinV: voltages,
    idealResistor: { spinH: 0, phaseDeg: 0, torqueStepT63Seconds: 0, breakawaySpinV: 0 } },
  checks: 'PASS: port balance, invalid drives, passive coast, reaction slope, odd DC, calibration, AC inertia and step refinement',
  cases,
};
const n = (x, digits = 2) => Number(x.toFixed(digits));
const at = (c, v) => c.dc.points.find(p => p.spinV === v);
console.log('Simulated vs ideal resistor. Resistance in spin Ω, inductance in spin H.');
console.table(cases.map(c => ({
  part: c.properties.kind === 'oil' ? `Oil ${c.idealSpinOhm}` : 'Eddy', source: c.source,
  'ideal R': c.idealSpinOhm, 'DC fit R': n(c.dc.fitSpinOhm),
  'R at 0.25 V': n(at(c, 0.25).secantSpinOhm), 'R at 6 V': n(at(c, 6).secantSpinOhm),
  '6 V current Δ%': n(at(c, 6).currentErrorPct), 'max |Δ%|': n(c.dc.maxCurrentErrorPct),
  'AC R at 3 V': n(c.ac[1].fitSpinOhm), 'AC L': n(c.ac[1].fitSpinH, 4),
  't63 (ms)': n(c.step.t63Seconds * 1000, 3),
})));
console.log('Ideal phase and t63 are zero. Measured impedance with 3 spin V DC bias:');
console.table(cases.flatMap(c => c.ac[1].points.filter(p => [0.5, 10, 100].includes(p.hz)).map(p => ({
  part: c.properties.kind === 'oil' ? `Oil ${c.idealSpinOhm}` : 'Eddy', hz: p.hz,
  '|Z| (spin Ω)': n(p.magnitudeSpinOhm), 'phase (deg)': n(p.phaseDeg),
}))));
console.log(results.checks);
writeFileSync(new URL('./results.json', import.meta.url), JSON.stringify(results, null, 2) + '\n');
