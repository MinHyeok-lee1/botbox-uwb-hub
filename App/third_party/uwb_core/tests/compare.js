'use strict';

const fs = require('node:fs');
const readline = require('node:readline');
const { spawn } = require('node:child_process');
const { once } = require('node:events');
const { PdoaResearchSession } = require('../../../src/pdoa_research_session');
const { flattenPdoaBundleResult } = require('../../../src/pdoa_bundle');
const { Pf13Input } = require('../../../src/pdoa_pf13_input');
const { PdoaObservationClock } = require('../../../src/pdoa_observation_clock');
const { projectTrackingContinuity } = require('../../../src/pdoa_tracking_continuity');

const parameters = Object.freeze({ consensus: .55, adaptiveCenterEnterDeg: 30, adaptiveDirectionEnterDeg: 50 });
const directions = ['UNKNOWN', 'LEFT', 'CENTER', 'RIGHT'];
const modes = ['UNKNOWN', 'FAST', 'SHORT', 'MEDIUM', 'LONG', 'HOLD', 'NORMAL', 'ROBUST', 'DEGRADED',
    'LOST', 'RECOVERY', 'TRACKING', 'SOFT HOLD', 'DEGRADED TRACKING', 'BOUNDARY'];
const engineKeys = ['intelligent', 'tracking', 'boundary'];
const num = n => Number.isFinite(n) ? String(n) : 'nan';
const bit = value => Number(Boolean(value));
const mode = value => modes.indexOf(value);
const direction = value => directions.indexOf(value);

class Oracle {
    constructor(write) {
        this.write = write;
        this.session = new PdoaResearchSession(parameters);
        // Stats are display-only. The estimator and pairing paths below are unchanged.
        this.session.stats = () => null;
        this.coverage = { v1Transition: 0, v1Lost: 0, v1Recovery: 0, v2Held: 0,
            v3Boundary: 0, v3Reference: 0, v2ExpiryEdges: 0 };
        this.checkedHold = false;
        this.reset();
    }

    reset() {
        this.session.clear();
        this.session.resetAlgorithms();
        this.clocks = engineKeys.map(() => new PdoaObservationClock(250));
        this.last = null;
        this.paired = null;
        this.write('X\n');
    }

    observe(frame, center, arrival) {
        const output = this.session.ingest(frame, { centerMdeg: center });
        if (output?.kind !== 'D') return false;
        this.session.d.length = 0;
        const r = output.result;
        this.paired = null;
        this.last = { frame, center, arrival };
        const input = { tick: frame.tick, seq: frame.seq, center: r.context.center,
            valid: frame.valid && !(frame.reason & 7) && frame.quality > r.parameters.qualityFloor };
        for (let i = 0; i < engineKeys.length; i++) {
            this.clocks[i].ingest(input, arrival);
            this.clocks[i].publish({ ...r, intelligent: r[engineKeys[i]] }, arrival);
        }

        const a = r.adaptive, t = r.tracking, b = r.boundary;
        const methods = flattenPdoaBundleResult(r, r.parameters);
        const values = methods.flatMap((m, i) => i < 5 ? [m.phase] :
            i < 13 ? [m.phase, m.sigma] : [m.phase, m.sigma, m.score, m.occupancy]);
        const discrete = [methods.map(m => direction(m.state)).join(''), mode(a.mode), mode(a.processingMode),
            mode(r.intelligent.mode), mode(t.mode), mode(b.mode), bit(t.isHeld), bit(b.isBoundary),
            bit(b.referenceUsed), t.heldMs, t.graceMs, bit(a.persistentOffset)];
        this.write(`D ${frame.tick} ${frame.seq} ${arrival} ${frame.raw} ${bit(frame.valid)} ${frame.quality} ${frame.reason} ${center ?? 0} ${bit(center !== null)}\n` +
            `E ${discrete.join(' ')} ${values.map(num).join(' ')}\n`);
        this.coverage.v1Transition += bit(r.intelligent.transition);
        this.coverage.v1Lost += bit(r.intelligent.mode === 'LOST');
        this.coverage.v1Recovery += bit(r.intelligent.mode === 'RECOVERY');
        this.coverage.v2Held += bit(t.isHeld);
        this.coverage.v3Boundary += bit(b.isBoundary);
        this.coverage.v3Reference += bit(b.referenceUsed);
        if (!this.checkedHold && t.isHeld) {
            const expires = arrival + t.graceMs - t.heldMs;
            this.poll(expires);
            this.poll(expires + 1);
            this.coverage.v2ExpiryEdges++;
            this.checkedHold = true;
        }
        return true;
    }

    pair(frame) {
        if (this.session.ingest(frame)?.kind !== 'P') return false;
        this.session.p.length = 0;
        this.paired = frame.distance >= 0 ? frame.distance : null;
        this.write(`P ${frame.tick} ${frame.seq} ${frame.distance} ${bit(frame.distance >= 0)} 0\n`);
        return true;
    }

    poll(now) {
        const last = this.last;
        const lost = last && now - last.arrival > 250;
        const distanceValid = this.paired !== null && !lost;
        this.clocks.forEach((clock, i) => {
            const view = clock.view(now);
            const value = i === 1 ? projectTrackingContinuity(view, now - (last?.arrival ?? now)) : view;
            const valid = value.state !== 'UNKNOWN';
            const origin = !valid ? 0 : value.isHeld ? 2 : value.referenceUsed ? 3 : 1;
            const degraded = ['DEGRADED', 'DEGRADED TRACKING', 'SOFT HOLD'].includes(value.mode);
            this.write(['Q', now, 14 + i, direction(value.state), mode(value.mode), origin, bit(last), bit(lost),
                bit(last && last.center !== null), bit(distanceValid), distanceValid ? this.paired : -1,
                last?.frame.seq ?? 0, last?.frame.tick ?? 0, last?.arrival ?? 0, bit(valid), bit(degraded)].join(' ') + '\n');
        });
    }
}

function exerciseEdges(oracle) {
    let arrival = 4294967000, tick = 200, sequence = 1;
    const raw = deg => Math.round(deg * Math.PI / 180 * 2048);
    function sample(phase, options = {}) {
        tick += options.dt ?? 40;
        arrival += options.dt ?? 40;
        sequence += options.skip ?? 1;
        const d = { kind: 'D', sn: sequence & 255, seq: sequence, tick, valid: options.valid ?? true,
            reason: options.reason ?? 0, raw: raw(phase), quality: options.quality ?? 250 };
        oracle.observe(d, options.center === undefined ? 0 : options.center, arrival);
        if (!options.noRange)
            oracle.pair({ ...d, kind: 'P', tick: tick + 7, distance: options.distance ?? 1990 });
        oracle.poll(arrival);
    }
    oracle.poll(arrival);
    for (let i = 0; i < 80; i++) sample([-179, 179, -173, 175, 178][i % 5]);
    for (let i = 0; i < 75; i++) sample([19, 21, 24, 16, 20][i % 5]);
    sample(20, { reason: 8, quality: 240, skip: 2, distance: -1 });
    sample(20, { center: null });
    sample(20, { valid: false, reason: 3, noRange: true });
    sample(20, { quality: 230 });
    sample(20, { distance: 2600 });
    oracle.poll(arrival + 250);
    oracle.poll(arrival + 251);
    sample(20, { dt: 251 });
    sample(20);
    sample(20);
    for (let i = 0; i < 10; i++) sample(0, { center: i < 5 ? 20000 : -20000 });
    // Match JS epoch resets on unsigned counter rollover.
    tick = 0xffffffff - 41;
    sequence = 0xffffffff - 2;
    sample(-20, { dt: 1 });
    sequence = -1;
    sample(-20, { dt: 1, noRange: true });
    tick = -1;
    sample(20, { dt: 1 });
    oracle.poll(arrival + 251);
}

async function replayCapture(path, oracle, flush, since) {
    oracle.reset();
    const input = new Pf13Input();
    const rows = readline.createInterface({ input: fs.createReadStream(path), crlfDelay: Infinity });
    let carry = '', observations = 0, pairs = 0, lastArrival = 0;
    for await (const line of rows) {
        const chunk = JSON.parse(line);
        if (since !== undefined) {
            const wallTime = Date.parse(chunk.host_wall_iso);
            if (!Number.isFinite(wallTime)) throw Error('The capture has no valid host_wall_iso timestamp');
            if (wallTime < since) continue;
        }
        const hostMs = chunk.host_monotonic_ns / 1e6;
        lastArrival = Math.trunc(hostMs);
        const records = (carry + Buffer.from(chunk.hex, 'hex').toString('latin1')).split('\n');
        carry = records.pop();
        for (const raw of records) {
            const result = input.ingest(raw, hostMs, true);
            if (!result) continue;
            if (result.reset) oracle.reset();
            if (result.frame?.kind === 'P' && result.paired && oracle.pair(result.frame)) ++pairs;
            if (result.frame?.kind === 'D' && result.observation &&
                oracle.observe(result.frame, input.centerMdeg, lastArrival)) ++observations;
        }
        await flush(false);
    }
    oracle.poll(lastArrival + 251);
    console.log(`capture=${path} observations=${observations} ranges=${pairs}`);
    if (!observations) throw Error('Golden capture contains no usable D records');
}

async function main(executable, args) {
    if (!executable)
        throw Error('Usage: node compare.js <uwb_replay executable> [--since ISO-8601] [COM5_chunks.jsonl ...]');
    const restricted = args[0] === '--since';
    const since = restricted ? Date.parse(args[1]) : undefined;
    if (restricted && (!Number.isFinite(since) || args.length < 3))
        throw Error('--since requires a timestamp and at least one capture');
    const paths = restricted ? args.slice(2) : args;
    const child = spawn(executable, [], { stdio: ['pipe', 'inherit', 'inherit'] });
    const completion = once(child, 'close');
    // Keep the pipe back-pressured without storing a whole capture.
    let pending = '';
    const oracle = new Oracle(text => { pending += text; });
    async function flush(force = true) {
        if (!force && pending.length < 65536) return;
        if (pending) await new Promise((resolve, reject) =>
            child.stdin.write(pending, error => error ? reject(error) : resolve()));
        pending = '';
    }
    child.stdin.on('error', error => {
        if (error.code !== 'EPIPE') console.error(error.message);
    });
    try {
        exerciseEdges(oracle);
        await flush();
        for (const path of paths) await replayCapture(path, oracle, flush, since);
        await flush();
        child.stdin.end();
        const [exit] = await completion;
        if (exit !== 0) throw Error(`C replay exited with status ${exit}`);
        console.log(`coverage=${JSON.stringify(oracle.coverage)}`);
    } finally {
        if (child.exitCode === null) child.stdin.end();
    }
}

const [, , executable, ...paths] = process.argv;
main(executable, paths).catch(error => { console.error(error.message); process.exitCode = 1; });
