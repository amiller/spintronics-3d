import assert from 'node:assert/strict';
import { writeFileSync } from 'node:fs';
import { create, defaults, KIT_CAPACITANCE, KIT_INDUCTANCE } from './model.js';

const h = defaults.fixedStep;
const tau = 2 * Math.PI;
const free = { torque: 0 };
const twoPort = create().properties;
const loaded = { capacitance: KIT_CAPACITANCE };
const { radius: r, g, r1, r2Drag: r2, l1, l2 } = twoPort;
const c = KIT_CAPACITANCE;
const idealSynthetic = g ** 2 * c;
const limitInductance = l1 + idealSynthetic;
const relative = (a, b) => Math.abs(a - b) / Math.abs(b);
const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
const slope = pairs => {
  const xMean = mean(pairs.map(([x]) => x));
  const yMean = mean(pairs.map(([, y]) => y));
  return pairs.reduce((sum, [x, y]) => sum + (x - xMean) * (y - yMean), 0)
    / pairs.reduce((sum, [x]) => sum + (x - xMean) ** 2, 0);
};

function dcSweep() {
  const points = [];
  for (const drivePort of [0, 1]) {
    for (const current of [-0.04, -0.02, -0.01, -0.002, 0, 0.002, 0.01, 0.02, 0.04]) {
      const gyro = create();
      const commands = [{ speed: 0 }, { speed: 0 }];
      commands[drivePort].speed = current / r;
      gyro.step(h, commands); // Remove acceleration from the sample.
      const readings = gyro.step(h, commands);
      points.push({ drivePort: drivePort + 1, current, voltage: readings.map(port => port.voltage) });
    }
  }
  const fits = [1, 2].map(port => [0, 1].map(i => {
    const pairs = points.filter(point => point.drivePort === port).map(p => [p.current, p.voltage[i]]);
    const gradient = slope(pairs);
    return {
      slope: gradient,
      intercept: mean(pairs.map(([x, y]) => y - gradient * x)),
      maxResidual: Math.max(...pairs.map(([x, y]) => Math.abs(y - gradient * x))),
    };
  }));
  const z11 = fits[0][0].slope;
  const z21 = fits[0][1].slope;
  const z12 = fits[1][0].slope;
  const z22 = fits[1][1].slope;
  return { points, fits, z11, z12, z21, z22, gyration: (z21 - z12) / 2 };
}

function stepResponse(capacitance, voltage, duration) {
  const gyro = create({ capacitance });
  const trace = [];
  const count = Math.round(duration / h);
  let peakTilt = 0;
  for (let n = 1; n <= count; n++) {
    gyro.step(h, [{ torque: voltage * r }, free]);
    const current = gyro.state.omega.map(omega => omega * r);
    peakTilt = Math.max(peakTilt, Math.abs(gyro.state.angle[1]));
    trace.push({
      time: n * h,
      current1: current[0],
      current2: current[1],
      ...(capacitance > 0 ? {
        springVoltage: -gyro.properties.spring * gyro.state.angle[1] / r,
        idealFlywheelCurrent: voltage * n * h / KIT_INDUCTANCE,
      } : {}),
    });
  }
  return { voltage, trace, peakTilt };
}

const complex = (re, im) => ({ re, im, magnitude: Math.hypot(re, im), phaseDeg: Math.atan2(im, re) * 180 / Math.PI });
function divide(a, b) {
  const norm = b.re ** 2 + b.im ** 2;
  return complex((a.re * b.re + a.im * b.im) / norm, (a.im * b.re - a.re * b.im) / norm);
}

function measure(frequency, test) {
  const gyro = create(test === 'synthetic' ? loaded : {});
  const sums = Array.from({ length: 4 }, () => ({ re: 0, im: 0 }));
  const warmup = Math.round(6 / h);
  // Integer Hz use two seconds. Fractions above 1 Hz share an eight-second period.
  const duration = frequency < 1 ? 2 / frequency : Number.isInteger(frequency) ? 2 : 8;
  const samples = Math.round(duration / h);
  let maxTilt = 0;
  for (let n = 0; n < warmup + samples; n++) {
    const angle = tau * frequency * (n + 0.5) * h;
    let commands;
    if (test === 'free') {
      commands = [{ torque: 0.002 * r * Math.sin(angle) }, free];
    } else {
      const speed = { speed: 0.0008 / r * Math.sin(tau * frequency * (n + 1) * h) };
      commands = test === 'synthetic' ? [speed, free]
        : test === 'z1' ? [speed, { speed: 0 }] : [{ speed: 0 }, speed];
    }
    const [p1, p2] = gyro.step(h, commands);
    maxTilt = Math.max(maxTilt, Math.abs(gyro.state.angle[1]));
    if (n < warmup) continue;
    const values = [p1.voltage, p2.voltage, p1.current, p2.current];
    sums.forEach((sum, index) => {
      sum.re += values[index] * Math.cos(angle);
      sum.im -= values[index] * Math.sin(angle);
    });
  }
  return { sums, maxTilt };
}

function frequencySweep() {
  return [0.125, 0.25, 0.5, 1, 2, 3, 3.125, 3.25, 3.5, 4, 8, 10, 12, 13, 14, 16, 24, 32, 48, 64].map(frequency => {
    const [p1, p2, loaded1, free1] = ['z1', 'z2', 'synthetic', 'free'].map(test => measure(frequency, test));
    const [v11, v21, i1] = p1.sums;
    const [v12, v22, , i2] = p2.sums;
    const [vin, , iin] = loaded1.sums;
    const [vFree, , , iFree] = free1.sums;
    const z = tau * frequency;
    const x = z * l2 - 1 / (z * c);
    const springBranch = divide({ re: g ** 2, im: 0 }, { re: r2, im: x });
    return {
      frequency,
      Z11: divide(v11, i1), Z12: divide(v12, i2),
      Z21: divide(v21, i1), Z22: divide(v22, i2),
      Y21Free: divide(iFree, vFree),
      synthetic: divide(vin, iin),
      formula: complex(r1 + springBranch.re, z * l1 + springBranch.im),
      idealSynthetic: complex(0, z * idealSynthetic),
      nominalKit: complex(0, z * KIT_INDUCTANCE),
      maxTilt: Math.max(p1.maxTilt, p2.maxTilt, loaded1.maxTilt, free1.maxTilt),
    };
  });
}

function verifyEnergy() {
  let worst = 0;
  for (const capacitance of [0, c]) {
    const gyro = create({ capacitance });
    for (let n = 0; n < 1000; n++) {
      const direct = { speed: 0.09 * Math.sin(n / 42) };
      const voltage = { torque: 1e-4 * Math.cos(n / 32) };
      const drive = [[direct, free], [free, direct], [voltage, free], [direct, direct]][n % 4];
      const old = gyro.state.energy;
      const [p1, p2] = gyro.step(h, drive);
      const power = p1.torque * p1.speed + p2.torque * p2.speed;
      const loss = defaults.drag1 * p1.speed ** 2 + defaults.drag2 * p2.speed ** 2;
      worst = Math.max(worst, Math.abs(gyro.state.energy - old - h * (power - loss)));
    }
  }
  return worst;
}

function verifyNetworkCompanion() {
  const tick = 1 / 480; // sim/codex/index.html:127
  let worst = 0;
  for (const capacitance of [0, c]) {
    const gyro = create({ fixedStep: tick, capacitance });
    const inverseCap = capacitance === 0 ? 0 : 1 / capacitance;
    const m11 = l1 / tick + r1 / 2;
    const m22 = l2 / tick + r2 / 2 + tick * inverseCap / 4;
    const m12 = -g / 2, m21 = g / 2;
    for (let n = 0; n < 960; n++) {
      const v1 = 0.02 * Math.cos(14 * n * tick);
      const v2 = capacitance === 0 ? 0.01 * Math.sin(20 * n * tick) : 0;
      const [i1, i2] = gyro.state.omega.map(omega => r * omega);
      const b1 = (m11 - 2 * l1 / tick) * i1 + m12 * i2;
      const b2 = m21 * i1 + (m22 - 2 * l2 / tick) * i2 + inverseCap * r * gyro.state.angle[1];
      const next1 = (v1 - b1 - m12 * (v2 - b2) / m22) / (m11 - m12 * m21 / m22);
      const next2 = (v2 - b2 - m21 * next1) / m22;
      gyro.step(tick, [{ torque: r * v1 }, { torque: r * v2 }]);
      worst = Math.max(worst, Math.abs(next1 - r * gyro.state.omega[0]), Math.abs(next2 - r * gyro.state.omega[1]));
    }
  }
  return worst;
}

const dc = dcSweep();
const bare = stepResponse(0, 0.003, 2.5);
const syntheticStep = stepResponse(c, 0.018, 0.6);
const dcCurrent2 = -bare.voltage * g / (g ** 2 + r1 * r2);
const peaks = bare.trace.filter((sample, i, trace) => i > 0 && i < trace.length - 1
  && sample.time < 0.65 && sample.current2 > trace[i - 1].current2 && sample.current2 > trace[i + 1].current2
  && sample.current2 > dcCurrent2);
assert.ok(peaks.length >= 3, 'Need at least three nutation peaks');
const frequency = (peaks.length - 1) / (peaks.at(-1).time - peaks[0].time);
const decay = -slope(peaks.map(p => [p.time, Math.log(p.current2 - dcCurrent2)]));
const predictedDecay = (r1 / l1 + r2 / l2) / 2;
const predictedFrequency = Math.sqrt(g ** 2 / (l1 * l2) - ((r1 / l1 - r2 / l2) / 2) ** 2) / tau;
const finalBare = bare.trace.at(-1);
const finalSynthetic = syntheticStep.trace.at(-1);
const sweep = frequencySweep();
const fittedL1 = mean(sweep.map(p => p.Z11.im / (tau * p.frequency)));
const fittedL2 = mean(sweep.map(p => p.Z22.im / (tau * p.frequency)));
const lowBand = sweep.filter(p => p.frequency <= 0.5);
function extrapolateToDC(value) {
  const pairs = lowBand.map(p => [(tau * p.frequency) ** 2, value(p)]);
  const gradient = slope(pairs);
  return mean(pairs.map(([x, y]) => y - gradient * x));
}
const fitSyntheticL = extrapolateToDC(p => p.synthetic.im / (tau * p.frequency));
const fitSyntheticR = extrapolateToDC(p => p.synthetic.re);
const fitCapacitance = (fitSyntheticL - fittedL1) / dc.gyration ** 2;
const peak = sweep.reduce((best, point) => point.synthetic.magnitude > best.synthetic.magnitude ? point : best);
const poleHz = 1 / (tau * Math.sqrt(l2 * c));
const maxSweptTilt = Math.max(...sweep.map(p => p.maxTilt));
const energyResidual = verifyEnergy();
const networkCompanionResidual = verifyNetworkCompanion();
const dcResidual = Math.max(...dc.fits.flat().map(fit => fit.maxResidual));

assert.ok(dc.z12 < 0 && dc.z21 > 0, 'Cross-port signs');
assert.ok(relative(dc.gyration, g) < 1e-10, 'DC gyration fit');
assert.ok(relative(dc.z11, r1) < 1e-10 && relative(dc.z22, r2) < 1e-10, 'DC drag fit');
assert.ok(dcResidual < 1e-12, 'DC linearity');
assert.ok(relative(frequency, predictedFrequency) < 0.002 && relative(decay, predictedDecay) < 0.002, 'Nutation fit');
assert.ok(relative(finalBare.current2, dcCurrent2) < 0.005, 'Bare steady current');
assert.ok(relative(fittedL1, l1) < 0.002 && relative(fittedL2, l2) < 0.002, 'Gimbal inductance fit');
assert.ok(relative(fitSyntheticL, limitInductance) < 0.002 && relative(fitCapacitance, c) < 0.002,
  `Synthetic L and C fit: ${fitSyntheticL} vs ${limitInductance} kg, ${fitCapacitance} vs ${c} m/N`);
assert.ok(relative(fitSyntheticR, r1) < 0.01, 'Synthetic DC resistance');
assert.ok(sweep.every(p => Math.abs(p.Z12.re + g) < 1e-10 && Math.abs(p.Z21.re - g) < 1e-10
  && Math.abs(p.Z12.im) < 1e-10 && Math.abs(p.Z21.im) < 1e-10), 'AC signed cross-impedance');
assert.ok(sweep.every(p => p.synthetic.re > 0 && relative(p.synthetic.magnitude, p.formula.magnitude) < 0.025),
  'Synthetic impedance against the continuous-time formula');
assert.ok(Math.max(maxSweptTilt, bare.peakTilt, syntheticStep.peakTilt) < 0.25,
  `Small-angle bench window: sweep ${maxSweptTilt}, free step ${bare.peakTilt}, loaded step ${syntheticStep.peakTilt} rad`);
assert.ok(energyResidual < 1e-16, 'Discrete work-energy identity');
assert.ok(networkCompanionResidual < 1e-14, 'Network companion endpoint currents');
const reverse = create({ spinRate: -defaults.spinRate });
reverse.step(h, [{ speed: 1 }, { speed: 0 }]);
assert.ok(relative(reverse.step(h, [{ speed: 1 }, { speed: 0 }])[1].voltage, -g * r) < 1e-10, 'Spin reversal');
assert.throws(() => reverse.step(h / 2, [free, free]), /multiple/);
assert.throws(() => reverse.step(h, [{ speed: 0, torque: 0 }, free]), /Supply/);
const longStep = create(loaded), shortSteps = create(loaded);
longStep.step(8 * h, [{ speed: 0.8 }, free]);
const stepTorques = [];
for (let i = 1; i <= 8; i++) stepTorques.push(shortSteps.step(h, [{ speed: 0.1 * i }, free])[0].torque);
assert.ok(Math.abs(longStep.state.energy - shortSteps.state.energy) < 1e-16
  && Math.abs(longStep.ports[0].torque - mean(stepTorques)) < 1e-14, 'Fixed-tick speed ramp and interval mean');

const results = {
  units: { voltage: 'N', current: 'm/s', impedance: 'N s/m', inductance: 'kg', capacitance: 'm/N' },
  parameters: twoPort,
  dc: { ...dc, maxResidual: dcResidual },
  step: {
    nutation: { frequency, decay, predictedFrequency, predictedDecay, dcCurrent2, final: finalBare,
      idealCurrent2: -bare.voltage / g, voltage: bare.voltage, peakTilt: bare.peakTilt,
      trace: bare.trace.filter((_, i) => (i + 1) % 16 === 0) },
    synthetic: { final: finalSynthetic, voltage: syntheticStep.voltage, peakTilt: syntheticStep.peakTilt,
      trace: syntheticStep.trace.filter((_, i) => (i + 1) % 16 === 0) },
  },
  frequencySweep: sweep,
  fits: { fittedL1, fittedL2, fitSyntheticL, fitSyntheticR, fitCapacitance, idealSynthetic,
    limitInductance, kitFlywheel: KIT_INDUCTANCE, peak, poleHz },
  verification: { energyResidual, networkCompanionResidual, maxSweptTilt, tests: 'passed' },
};

const num = value => Number(value.toPrecision(6));
console.table([
  ['Z11 DC (N s/m)', dc.z11, 0],
  ['Z12 DC (N s/m)', dc.z12, -g],
  ['Z21 DC (N s/m)', dc.z21, g],
  ['Z22 DC (N s/m)', dc.z22, 0],
  ['Gyration G (N s/m)', dc.gyration, g],
  ['Axis 1 / 2 extra inertia (kg)', `${num(fittedL1)} / ${num(fittedL2)}`, '0 / 0'],
  ['Nutation (Hz)', frequency, '—'],
  ['Nutation decay (1/s)', decay, '—'],
  ['Synthetic L, low-f limit (kg)', fitSyntheticL, idealSynthetic],
  ['Kit flywheel, nominal (kg)', KIT_INDUCTANCE, KIT_INDUCTANCE],
  ['Synthetic series R, low-f limit (N s/m)', fitSyntheticR, 0],
  ['Inferred load C (m/N)', fitCapacitance, c],
  ['Synthetic I at 0.6 s (m/s)', finalSynthetic.current1, syntheticStep.voltage * 0.6 / idealSynthetic],
  ['Kit flywheel I at 0.6 s (m/s)', finalSynthetic.idealFlywheelCurrent, finalSynthetic.idealFlywheelCurrent],
].map(([quantity, measured, ideal]) => ({
  quantity, model: typeof measured === 'number' ? num(measured) : measured,
  ideal: typeof ideal === 'number' ? num(ideal) : ideal,
})));
console.table(sweep.map(p => ({
  'Hz': p.frequency, '|Z21|': num(p.Z21.magnitude), '∠Z21 (°)': num(p.Z21.phaseDeg),
  '|Zsynthetic|': num(p.synthetic.magnitude), '∠Zsynthetic (°)': num(p.synthetic.phaseDeg),
  '|Zkit|': num(p.nominalKit.magnitude), '∠Zkit (°)': 90,
})));
console.log(`All checks passed. Worst work-energy residual ${energyResidual.toExponential(2)} J per step.`);
writeFileSync(new URL('./results.json', import.meta.url), `${JSON.stringify(results, null, 2)}\n`);
