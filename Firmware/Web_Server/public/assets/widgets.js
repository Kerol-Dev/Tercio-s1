// -----------------------------------------------------------------------------
// Canvas widgets: the shaft dial and the live strip chart. Both read their
// colours from the CSS theme tokens, so they follow light/dark switching.
// -----------------------------------------------------------------------------

const TAU = Math.PI * 2;

// Colours are read lazily on the next draw: a canvas that is not in the
// document yet has no computed style.
function colorsFor(widget) {
  if (!widget.colors) {
    const colors = themeColors(widget.canvas);
    if (!colors.ink) return colors;  // detached: try again next frame
    widget.colors = colors;
  }
  return widget.colors;
}

export function themeColors(element) {
  const css = getComputedStyle(element);
  const get = name => css.getPropertyValue(name).trim();
  return {
    ink: get('--ink'), ink2: get('--ink-2'), ink3: get('--ink-3'), line: get('--line'), lineStrong: get('--line-strong'),
    panel: get('--panel'), panel2: get('--panel-2'), accent: get('--accent'), good: get('--good'), warn: get('--warn'),
    bad: get('--bad'), busy: get('--busy'), mono: get('--font-mono'), ui: get('--font-ui'),
  };
}

function fitCanvas(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const { width, height } = canvas.getBoundingClientRect();
  const w = Math.max(1, Math.round(width * ratio));
  const h = Math.max(1, Math.round(height * ratio));
  if (canvas.width !== w || canvas.height !== h) {
    canvas.width = w;
    canvas.height = h;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, width, height };
}

// ---- Shaft dial -----------------------------------------------------------------------
// One revolution around the disc, 0 at the top, clockwise positive. The needle
// is the measured shaft angle, the notch outside the scale is the target.
export class Dial {
  constructor(canvas) {
    this.canvas = canvas;
    this.colors = null;
  }

  refreshTheme() {
    this.colors = null;
  }

  // turns: measured position; target: target position (both in turns)
  draw({ turns = 0, target = 0, tone = 'neutral', active = false, labels = ['0', '90', '180', '270'] } = {}) {
    const { ctx, width, height } = fitCanvas(this.canvas);
    const c = colorsFor(this);
    const size = Math.min(width, height);
    const cx = width / 2;
    const cy = height / 2;
    const r = size / 2 - 22;
    ctx.clearRect(0, 0, width, height);

    // Disc
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, TAU);
    ctx.fillStyle = c.panel2;
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = c.line;
    ctx.stroke();

    // Degree scale: 72 ticks, longer every 30°, longest at the quarters.
    for (let i = 0; i < 72; i++) {
      const a = (i / 72) * TAU - Math.PI / 2;
      const major = i % 6 === 0;
      const quarter = i % 18 === 0;
      const inner = r - (quarter ? 14 : major ? 10 : 5);
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(a) * inner, cy + Math.sin(a) * inner);
      ctx.lineTo(cx + Math.cos(a) * (r - 1), cy + Math.sin(a) * (r - 1));
      ctx.lineWidth = quarter ? 1.6 : 1;
      ctx.strokeStyle = quarter ? c.ink2 : major ? c.ink3 : c.lineStrong;
      ctx.stroke();
    }
    ctx.fillStyle = c.ink3;
    ctx.font = `500 10px ${c.mono}`;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    labels.forEach((label, i) => {
      const a = (i / 4) * TAU - Math.PI / 2;
      const d = r + 12;
      ctx.fillText(label, cx + Math.cos(a) * d, cy + Math.sin(a) * d);
    });

    const angle = t => ((((t % 1) + 1) % 1) * TAU) - Math.PI / 2;
    const toneColor = { good: c.good, accent: c.accent, busy: c.busy, bad: c.bad, warn: c.warn }[tone] ?? c.ink3;

    // Travel arc from target to measured (the following error, visible when it matters).
    const error = target - turns;
    if (active && Math.abs(error) > 0.002) {
      ctx.beginPath();
      const a0 = angle(turns);
      const sweep = Math.max(-TAU, Math.min(TAU, error * TAU));
      ctx.arc(cx, cy, r - 20, a0, a0 + sweep, sweep < 0);
      ctx.lineWidth = 4;
      ctx.lineCap = 'round';
      ctx.strokeStyle = toneColor;
      ctx.globalAlpha = 0.35;
      ctx.stroke();
      ctx.globalAlpha = 1;
    }

    // Target notch
    if (active) {
      const a = angle(target);
      const tip = r + 1;
      ctx.beginPath();
      ctx.moveTo(cx + Math.cos(a) * tip, cy + Math.sin(a) * tip);
      ctx.lineTo(cx + Math.cos(a - 0.06) * (tip + 9), cy + Math.sin(a - 0.06) * (tip + 9));
      ctx.lineTo(cx + Math.cos(a + 0.06) * (tip + 9), cy + Math.sin(a + 0.06) * (tip + 9));
      ctx.closePath();
      ctx.fillStyle = c.accent;
      ctx.fill();
    }

    // Needle: the outer part of the radius only, so the centre stays clear for
    // the digital readout.
    const a = angle(turns);
    const inner = r * 0.5;
    ctx.beginPath();
    ctx.moveTo(cx + Math.cos(a) * inner, cy + Math.sin(a) * inner);
    ctx.lineTo(cx + Math.cos(a) * (r - 16), cy + Math.sin(a) * (r - 16));
    ctx.lineWidth = 3;
    ctx.lineCap = 'round';
    ctx.strokeStyle = toneColor;
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(cx + Math.cos(a) * (r - 16), cy + Math.sin(a) * (r - 16), 4.5, 0, TAU);
    ctx.fillStyle = toneColor;
    ctx.fill();

    // Centre face
    ctx.beginPath();
    ctx.arc(cx, cy, inner - 6, 0, TAU);
    ctx.fillStyle = c.panel;
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = c.line;
    ctx.stroke();
  }
}

// ---- Strip chart -------------------------------------------------------------------------
// Ring buffer of samples; draws the last `window` seconds of one or two series.
export class History {
  constructor(capacity = 8000) {  // 80 s at 100 Hz, 30 s up to 250 Hz
    this.capacity = capacity;
    this.t = new Float64Array(capacity);
    this.series = { position: new Float64Array(capacity), target: new Float64Array(capacity), velocity: new Float32Array(capacity), error: new Float32Array(capacity) };
    this.head = 0;
    this.count = 0;
  }
  push(t, sample) {
    const i = this.head;
    this.t[i] = t;
    for (const key in this.series) this.series[key][i] = sample[key];
    this.head = (i + 1) % this.capacity;
    this.count = Math.min(this.count + 1, this.capacity);
  }
  *range(since) {
    for (let k = 0; k < this.count; k++) {
      const i = (this.head - this.count + k + this.capacity) % this.capacity;
      if (this.t[i] >= since) yield i;
    }
  }
}

function niceStep(span, target = 5) {
  const raw = span / target;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const norm = raw / mag;
  return (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
}

export class StripChart {
  constructor(canvas) {
    this.canvas = canvas;
    this.colors = null;
  }

  refreshTheme() {
    this.colors = null;
  }

  // lines: [{ key, color, fill, label }], scale: value multiplier, unit: axis unit label
  draw(history, now, { windowS = 10, lines, scale = 1, unit = '', minSpan = 0.01 }) {
    const { ctx, width, height } = fitCanvas(this.canvas);
    const c = colorsFor(this);
    const pad = { l: 58, r: 14, t: 12, b: 24 };
    const w = width - pad.l - pad.r;
    const h = height - pad.t - pad.b;
    ctx.clearRect(0, 0, width, height);
    if (w <= 0 || h <= 0) return;

    const since = now - windowS * 1000;
    const indices = [...history.range(since)];
    let lo = Infinity;
    let hi = -Infinity;
    for (const i of indices) {
      for (const line of lines) {
        const v = history.series[line.key][i] * scale;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    if (!Number.isFinite(lo)) {
      lo = -1;
      hi = 1;
    }
    const mid = (lo + hi) / 2;
    const span = Math.max(hi - lo, minSpan) * 1.15;
    lo = mid - span / 2;
    hi = mid + span / 2;
    const y = v => pad.t + h - ((v - lo) / (hi - lo)) * h;
    const x = t => pad.l + ((t - since) / (windowS * 1000)) * w;

    // Grid and axes
    const step = niceStep(hi - lo);
    ctx.font = `500 10.5px ${c.mono}`;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    const digits = Math.max(0, -Math.floor(Math.log10(step)));
    for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
      const py = Math.round(y(v)) + 0.5;
      ctx.strokeStyle = Math.abs(v) < step / 1e6 ? c.lineStrong : c.line;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(pad.l, py);
      ctx.lineTo(pad.l + w, py);
      ctx.stroke();
      ctx.fillStyle = c.ink3;
      ctx.fillText(v.toFixed(Math.min(digits, 4)), pad.l - 8, py);
    }
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let s = 0; s <= windowS; s += windowS / 5) {
      const px = pad.l + (s / windowS) * w;
      ctx.fillStyle = c.ink3;
      ctx.fillText(s === windowS ? 'now' : `−${windowS - s}s`, px, pad.t + h + 7);
    }
    if (unit) {
      ctx.save();
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillStyle = c.ink3;
      ctx.fillText(unit, 6, pad.t);
      ctx.restore();
    }

    // Series
    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.l, pad.t, w, h);
    ctx.clip();
    for (const line of lines) {
      if (indices.length < 2) break;
      ctx.beginPath();
      indices.forEach((i, k) => {
        const px = x(history.t[i]);
        const py = y(history.series[line.key][i] * scale);
        if (k === 0) ctx.moveTo(px, py);
        else ctx.lineTo(px, py);
      });
      ctx.lineWidth = line.width ?? 1.75;
      ctx.lineJoin = 'round';
      ctx.setLineDash(line.dash ?? []);
      ctx.strokeStyle = line.color;
      ctx.stroke();
      ctx.setLineDash([]);
      if (line.fill) {
        const last = indices.at(-1);
        ctx.lineTo(x(history.t[last]), pad.t + h);
        ctx.lineTo(x(history.t[indices[0]]), pad.t + h);
        ctx.closePath();
        const gradient = ctx.createLinearGradient(0, pad.t, 0, pad.t + h);
        gradient.addColorStop(0, line.color);
        gradient.addColorStop(1, 'transparent');
        ctx.globalAlpha = 0.12;
        ctx.fillStyle = gradient;
        ctx.fill();
        ctx.globalAlpha = 1;
      }
      const last = indices.at(-1);
      ctx.beginPath();
      ctx.arc(x(history.t[last]), y(history.series[line.key][last] * scale), 3, 0, TAU);
      ctx.fillStyle = line.color;
      ctx.fill();
    }
    ctx.restore();
  }
}
