import assert from 'node:assert/strict';
import {writeFileSync} from 'node:fs';
import {CHAIN, KIT, PARTS, create, engagement} from './model.js';

const TAU = 2 * Math.PI;
const h = 1 / 24000;
const soloH = 1 / 4800;
const rC = PARTS.capacitor.radius;
const rL = PARTS.flywheel.radius;
const fIdeal = 1 / (TAU * Math.sqrt(KIT.capacitance * KIT.inductance));
const freq = [0.25, 0.5, 1, 2, 5, 10, 20, 40, 80, 160];
const currents = [-0.16, -0.10, -0.04, 0, 0.04, 0.10, 0.16];

const round = (v, digits = 7) => Number(v.toPrecision(digits));
function near(actual, expected, tolerance) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} differs from ${expected} by more than ${tolerance}`);
}

function isolated(kind, overrides = {}) {
  return create({left: {kind, ...overrides}, right: null, h: soloH});
}

function chainRig(overrides = {}) {
  return create({left: 'idler', right: 'idler', chain: {polygonal: false, ...overrides}, h});
}

function speed(mode, current, radius) {
  const left = {speed: current / radius};
  if (mode === 'common') return {left, right: {speed: current / radius}};
  if (mode === 'differential') return {left: {speed: left.speed / 2}, right: {speed: -left.speed / 2}};
  return {left};
}

function chainEffort(port) {
  return port.voltage - PARTS.idler.inertia / port.radius * port.acceleration;
}

function frequency(factory, mode, frequencies, amplitude) {
  return frequencies.map(f => {
    const model = factory();
    const radius = model.ports.left.radius;
    const omega = TAU * f;
    const twoDriven = mode === 'common' || mode === 'differential';
    const dt = model.state.chain ? 8 * model.state.h : model.state.h;
    const settle = Math.ceil(Math.max(.6, 8 / f) / dt);
    const end = settle + Math.round(8 / (f * dt));
    const drive = t => speed(mode, amplitude * Math.sin(omega * t), radius);
    const totals = Array(8).fill(0);
    for (let j = 0; j < end; j++) {
      const {left, right} = model.step(dt, drive);
      const s = Math.sin(omega * model.state.time);
      const c = Math.cos(omega * model.state.time);
      // The drive is known at every tick, including the tick before this sample.
      let v = left.voltage;
      if (twoDriven) {
        const previous = drive(model.state.time - model.state.h);
        const vl = chainEffort({...left, acceleration: (left.speed - previous.left.speed) / model.state.h});
        const vr = chainEffort({...right, acceleration: (right.speed - previous.right.speed) / model.state.h});
        v = mode === 'common' ? vl + vr : (vl - vr) / 2;
      }
      if (j < settle) continue;
      totals[0] += s * s;
      totals[1] += c * c;
      totals[2] += s * c;
      totals[3] += s * v;
      totals[4] += c * v;
      totals[5] += s;
      totals[6] += c;
      totals[7] += v;
    }
    // Remove DC and solve the sine/cosine projection even for partial samples at cycle ends.
    let [ss, cc, sc, sv, cv, sumS, sumC, sumV] = totals;
    const samples = end - settle;
    ss -= sumS ** 2 / samples;
    cc -= sumC ** 2 / samples;
    sc -= sumS * sumC / samples;
    sv -= sumS * sumV / samples;
    cv -= sumC * sumV / samples;
    const determinant = ss * cc - sc ** 2;
    const real = (sv * cc - cv * sc) / determinant / amplitude;
    const imag = (cv * ss - sv * sc) / determinant / amplitude;
    assert.ok(Number.isFinite(real) && Number.isFinite(imag) && Math.abs(sumS) + Math.abs(sumC) < 4);
    return {Hz: f, real, imag, magnitude: Math.hypot(real, imag), phaseDeg: Math.atan2(imag, real) * 180 / Math.PI};
  });
}

function fitLine(points) {
  const meanX = points.reduce((sum, [x]) => sum + x, 0) / points.length;
  const meanY = points.reduce((sum, [, y]) => sum + y, 0) / points.length;
  let xy = 0, xx = 0;
  for (const [x, y] of points) {
    xy += (x - meanX) * (y - meanY);
    xx += (x - meanX) ** 2;
  }
  const slope = xy / xx;
  return {slope, intercept: meanY - slope * meanX};
}

function dcSolo(kind) {
  return currents.map(I => {
    const model = isolated(kind);
    const {left} = model.step(.24, speed('single', I, model.ports.left.radius));
    const shaft = model.state.shafts.left;
    return {I, V: left.voltage, angle: shaft.angle, springV: (shaft.p.spring ?? 0) * shaft.angle / left.radius,
      hystV: shaft.hysteresis / left.radius};
  });
}

const dc = {
  capacitorRamps: dcSolo('capacitor'),
  capacitorHold: [-.08, -.04, 0, .04, .08].map(x => {
    const model = isolated('capacitor', {angle: x / rC});
    model.step(soloH, {left: {speed: 0}});
    return {x, I: 0, V: model.ports.left.voltage, stictionHalfWidth: PARTS.capacitor.friction / rC};
  }),
  flywheel: dcSolo('flywheel'), motor: dcSolo('motor'), inerter: dcSolo('inerter'),
  chainCommon: currents.map(I => {
    const model = chainRig();
    model.step(.5, t => speed('common', I * (.5 - .5 * Math.cos(TAU * t)), rL));
    const drive = speed('common', I, rL);
    model.step(4, drive);
    let effort = 0;
    for (let j = 0; j < 500; j++) {
      const {left, right} = model.step(.002, drive);
      effort += left.voltage + right.voltage;
    }
    return {I, V: effort / 500};
  }),
  chainHold: [-.002, -.001, 0, .001, .002].map(dx => {
    const model = chainRig();
    model.step(.25, speed('differential', dx / .25, rL));
    const {left, right} = model.step(.75, speed('differential', 0, rL));
    return {dx, I: 0, V: (left.voltage - right.voltage) / 2};
  }),
};
const fitDrag = (rows, adjust = row => row.V) => fitLine(rows.filter(row => row.I > 0).map(row => [row.I, adjust(row)]));
const capDrag = fitDrag(dc.capacitorRamps, row => row.V - row.springV);
const wheelDrag = fitDrag(dc.flywheel);
const motorDrag = fitLine(dc.motor.map(row => [row.I, row.V - Math.sign(row.I) * PARTS.motor.friction / rL]));
const inerterDrag = fitDrag(dc.inerter);
const chainDrag = fitLine(dc.chainCommon.map(row => [row.I, row.V]));
const capC = 1 / fitLine(dc.capacitorHold.map(row => [row.x, row.V])).slope;
const chainC = 1 / fitLine(dc.chainHold.map(row => [row.dx, row.V])).slope;

const amplitudes = {shaft: .01, chainCommon: .015, chainDifferential: .0008};
const sweeps = {
  capacitor: frequency(() => isolated('capacitor'), 'single', freq, amplitudes.shaft),
  flywheel: frequency(() => isolated('flywheel'), 'single', freq, amplitudes.shaft),
  motor: frequency(() => isolated('motor'), 'single', freq, amplitudes.shaft),
  inerter: frequency(() => isolated('inerter'), 'single', freq, amplitudes.shaft),
  chainCommon: frequency(chainRig, 'common', freq, amplitudes.chainCommon),
  chainDifferential: frequency(chainRig, 'differential', freq, amplitudes.chainDifferential),
  assembledLC: frequency(() => create({h}), 'single', [.25, .4, .5, .55, .6, .625, .65, .75, 1, 2, 5], amplitudes.shaft),
};
const fitInertia = (rows, low = 1, high = 10) => fitLine(rows.filter(({Hz}) => Hz >= low && Hz <= high)
  .map(({Hz, imag}) => [TAU * Hz, imag])).slope;
const chainL = fitInertia(sweeps.chainCommon, .25, 2);
const wheelL = fitInertia(sweeps.flywheel);
const motorL = fitInertia(sweeps.motor);
const capL = fitInertia(sweeps.capacitor.map(row => ({...row, imag: row.imag + 1 / (TAU * row.Hz * capC)})), 40, 160);
const gears = [.5, 1, 2].map(ratio => {
  const rows = ratio === 1 ? sweeps.inerter : frequency(() => isolated('inerter', {ratio}), 'single', [1, 2, 5], amplitudes.shaft);
  return {ratio, measuredL: fitInertia(rows), analyticL: PARTS.inerter.inertia / rL ** 2 + PARTS.inerter.screwInertia * (TAU * ratio / PARTS.inerter.lead) ** 2, rows};
});

function stepTrace(model, drive, duration, interval, probe = m => ({I: m.ports.left.current, V: m.ports.left.voltage})) {
  return Array.from({length: Math.round(duration / interval)}, () => {
    model.step(interval, drive);
    return {t: model.state.time, ...probe(model)};
  });
}
const steps = {
  capacitor: stepTrace(isolated('capacitor'), {left: {torque: .3 * rC}}, 2, .0025,
    m => ({I: m.ports.left.current, V: m.ports.left.voltage, x: m.ports.left.angle * rC})),
  flywheel: stepTrace(isolated('flywheel'), {left: {torque: .1 * rL}}, 1, .005),
  inerter: stepTrace(isolated('inerter'), {left: {torque: .1 * rL}}, 1, .005),
  chainDifferential: stepTrace(chainRig(), {left: {torque: .25 * rL}, right: {speed: 0}}, 20, .005,
    m => ({I: m.ports.left.current, V: m.ports.left.voltage, x: m.ports.left.angle * rL}))
    .filter(({t}, i) => t <= 1 || (i + 1) % 20 === 0),
};
const motor = isolated('motor');
let breaker = null;
steps.motorUnloaded = stepTrace(motor, (t, ports) => {
  if (motor.state.shafts.left.tripped && !breaker) breaker = {time: motor.state.time, speed: ports.left.speed, current: ports.left.current};
  return {};
}, .6, .00125, m => ({I: m.ports.left.current, V: m.ports.left.voltage, tripped: m.state.shafts.left.tripped}));
assert.ok(breaker, 'Unloaded motor should trip');
assert.ok(breaker.speed >= PARTS.motor.tripSpeed);
motor.rewind();
assert.equal(motor.state.shafts.left.tripped, false);
assert.equal(motor.state.shafts.left.remaining, PARTS.motor.capacity);
const depletionRig = isolated('motor', {capacity: .02});
depletionRig.step(.05, {left: {speed: 1}});
assert.equal(depletionRig.state.shafts.left.remaining, 0);
assert.ok(depletionRig.ports.left.torque > 0, 'Depleted motor should require positive applied drive');

function chordal() {
  const model = create({left: 'capacitor', right: 'flywheel', h});
  const feed = .18;
  const rpm = Object.values(model.state.shafts).map(({p}) => feed * TAU / (CHAIN.pitch * p.teeth));
  const force = {left: {speed: rpm[0]}, right: {speed: rpm[1]}};
  const toothHz = feed / CHAIN.pitch;
  const time = Math.round(45 / (toothHz * h));
  const channels = Object.values(model.ports).map(() => ({min: Infinity, max: -Infinity, total: 0, s: 0, c: 0}));
  model.step(.48, force);
  for (let j = 0; j < time; j++) {
    model.step(h, force);
    const angle = TAU * toothHz * model.state.time;
    Object.values(model.ports).forEach(({feedSpeed}, i) => {
      const ch = channels[i];
      ch.min = Math.min(ch.min, feedSpeed);
      ch.max = Math.max(ch.max, feedSpeed);
      ch.total += feedSpeed;
      ch.s += (feedSpeed - feed) * Math.sin(angle);
      ch.c += (feedSpeed - feed) * Math.cos(angle);
    });
  }
  return {toothHz, feed, sprockets: channels.map((ch, i) => {
    const {p} = Object.values(model.state.shafts)[i];
    const mean = ch.total / time;
    const max = engagement(0, p.teeth, CHAIN.pitch).arm * rpm[i];
    return {teeth: p.teeth, mean, p2pPct: 100 * (ch.max - ch.min) / mean,
      geometricP2pPct: 100 * max * (1 - Math.cos(Math.PI / p.teeth)) / feed,
      toothFundamentalAmplitude: 2 * Math.hypot(ch.s, ch.c) / time};
  })};
}

function ringDown({polygonal = true, alternate = false, fine = false, lossless = false, preload = CHAIN.preload} = {}) {
  const initialV = KIT.voltage;
  const losses = lossless ? {drag: 0, friction: 0} : {};
  const left = {kind: 'capacitor', angle: initialV * KIT.capacitance / rC, ...losses};
  if (lossless) left.hysteresisTorque = 0;
  const right = {kind: alternate ? 'inerter' : 'flywheel', ...losses};
  const model = create({left, right, chain: {polygonal, preload, ...(lossless ? {damping: 0, linkDrag: 0} : {})}, h: fine ? h / 2 : h});
  let minTension = Infinity;
  let slackSamples = 0;
  const data = stepTrace(model, {}, 9, .002, m => {
    if (m.state.time <= 2) {
      let currentMin = Infinity;
      for (const span of m.state.strands) currentMin = Math.min(currentMin, ...span.tension);
      minTension = Math.min(minTension, currentMin);
      if (currentMin === 0) slackSamples++;
    }
    return {x: rC * m.ports.left.angle, I: m.ports.left.current};
  });
  const peaks = [];
  for (let i = 1; i < data.length - 1; i++) {
    const [a, b, c] = [data[i - 1], data[i], data[i + 1]];
    if (b.x <= a.x || b.x <= c.x || b.x < .002 || b.t < .5 || b.t - (peaks.at(-1)?.t ?? 0) < .8) continue;
    const curvature = a.x - 2 * b.x + c.x;
    const shift = .5 * (a.x - c.x) / curvature;
    peaks.push({t: b.t + shift * .002, x: b.x - .25 * (a.x - c.x) * shift});
  }
  assert.ok(peaks.length >= 4, 'Expected at least four resolved positive peaks');
  const periods = peaks.slice(1).map((v, i) => v.t - peaks[i].t);
  const decrements = peaks.slice(1).map((v, i) => Math.log(peaks[i].x / v.x));
  const period = periods.slice(0, 3).reduce((a, b) => a + b) / 3;
  const Q = decrements.map(d => lossless ? null : Math.hypot(TAU, d) / (2 * d));
  return {Hz: 1 / period, idealHz: fIdeal, deviationPct: 100 * (1 / period / fIdeal - 1), minTensionFirst2s: minTension,
    initialV, preload, slackSamplesFirst2s: slackSamples, Q, peaks, periods, decrements, trace: data.filter((_, i) => i % 10 === 0)};
}
const ripple = chordal();
const oscillator = {
  full: ringDown(), circular: ringDown({polygonal: false}), geared: ringDown({alternate: true}),
  losslessCircular: ringDown({polygonal: false, lossless: true, preload: .65}),
  taut: ringDown({preload: .65}), halfStep: ringDown({fine: true}),
};

near(capC, KIT.capacitance, 1e-10);
near(chainC, CHAIN.links / (2 * CHAIN.stiffness), 1e-5);
near(chainL, 2 * CHAIN.links * CHAIN.linkMass, .00004);
near(chainDrag.slope, 2 * CHAIN.links * CHAIN.linkDrag, .00002);
near(steps.chainDifferential.at(-1).x, .25 * chainC, .00002);
near(steps.chainDifferential.at(-1).I, 0, .003);
near(wheelL, KIT.inductance, .0001);
near(oscillator.full.Hz, oscillator.halfStep.Hz, .002);
assert.ok(oscillator.full.slackSamplesFirst2s > 0 && oscillator.taut.minTensionFirst2s > 0, 'Expected sampled slack at the lower preload and positive tension at the higher preload');
for (const gear of gears) near(gear.measuredL, gear.analyticL, .0001);
for (const p of ripple.sprockets) near(p.p2pPct, p.geometricP2pPct, .015);
const slack = chainRig({preload: 0});
slack.step(.02, speed('differential', .1, rL));
assert.ok(slack.state.strands.some(s => s.tension.every(t => t === 0)));

const adapter = isolated('capacitor');
for (const V of [0, .0003, .12, -.04, .9, 0, -.05]) {
  const stamp = adapter.companion(V);
  near(stamp.conductance * V + stamp.history, stamp.current, 1e-12);
  adapter.step(soloH, {left: {torque: V * rC}});
  near(adapter.ports.left.current, stamp.current, 1e-12);
}

const comparison = [
  {quantity: 'Capacitor C (m/N)', fit: capC, ideal: KIT.capacitance},
  {quantity: 'Capacitor rotor L (kg)', fit: capL, ideal: 0},
  {quantity: 'Capacitor viscous R (N s/m)', fit: capDrag.slope, ideal: 0},
  {quantity: 'Capacitor sliding + hysteresis V (N)', fit: capDrag.intercept, ideal: 0},
  {quantity: 'Flywheel L (kg)', fit: wheelL, ideal: KIT.inductance},
  {quantity: 'Flywheel viscous R (N s/m)', fit: wheelDrag.slope, ideal: 0},
  {quantity: 'Flywheel sliding V (N)', fit: wheelDrag.intercept, ideal: 0},
  {quantity: 'Motor source V (N)', fit: -motorDrag.intercept, ideal: KIT.voltage},
  {quantity: 'Motor rotor L (kg)', fit: motorL, ideal: 0},
  {quantity: 'Motor viscous R (N s/m)', fit: motorDrag.slope, ideal: 0},
  {quantity: 'Motor breaker nominal I (m/s)', fit: breaker.current, ideal: null},
  {quantity: 'Inerter L (kg), ratio 1', fit: gears[1].measuredL, ideal: KIT.inductance},
  {quantity: 'Inerter viscous R (N s/m)', fit: inerterDrag.slope, ideal: 0},
  {quantity: 'Chain differential C (m/N)', fit: chainC, ideal: 0},
  {quantity: 'Chain common L (kg)', fit: chainL, ideal: 0},
  {quantity: 'Chain common R (N s/m)', fit: chainDrag.slope, ideal: 0},
  {quantity: 'LC ring-down f (Hz)', fit: oscillator.full.Hz, ideal: fIdeal},
  {quantity: 'LC initial measured Q', fit: oscillator.full.Q[0], ideal: 'Infinity'},
];
const results = {units: {V: 'N', I: 'm/s', R: 'N s/m', C: 'm/N', L: 'kg'}, parameters: {CHAIN, PARTS, KIT},
  timings: {h, soloH}, amplitudes, comparison, dc, steps, sweeps, gears, breaker, ripple, oscillator};
writeFileSync(new URL('./results.json', import.meta.url), JSON.stringify(results, (key, value) => typeof value === 'number' ? round(value) : value, 2) + '\n');
console.table(comparison.map(row => ({...row, fit: round(row.fit)})));
console.table(Object.entries(oscillator).map(([variant, row]) => ({variant, Hz: round(row.Hz), Q1: row.Q[0] === null ? null : round(row.Q[0])})));
console.table(ripple.sprockets.map(row => ({teeth: row.teeth, 'p-p %': round(row.p2pPct), 'tooth Hz': ripple.toothHz})));
console.log(`Unloaded breaker: ${round(breaker.time)} s, ${round(breaker.speed)} rad/s. Assertions passed.`);
