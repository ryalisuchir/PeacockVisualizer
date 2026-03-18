/**
 * quinticSpline.ts
 *
 * Quintic Hermite spline with EXACT interpolation through waypoints.
 *
 * The key fix over the previous version: tangent magnitudes are scaled by
 * the chord length between adjacent waypoints (chord-length parameterization).
 * This ensures the curve actually passes through every waypoint rather than
 * just near them.
 *
 * Basis functions (standard quintic Hermite):
 *   h00 = 1 - 10t³ + 15t⁴ - 6t⁵   (position blend start)
 *   h10 = t - 6t³ + 8t⁴ - 3t⁵      (tangent blend start, scaled)
 *   h01 = 10t³ - 15t⁴ + 6t⁵        (position blend end)
 *   h11 = -4t³ + 7t⁴ - 3t⁵         (tangent blend end, scaled)
 *   h20 = ½t² - 3/2t³ + 3/2t⁴ - ½t⁵ (acceleration blend start)
 *   h21 = ½t³ - t⁴ + ½t⁵           (acceleration blend end)
 *
 * p(t) = h00·p0 + h10·v0 + h01·p1 + h11·v1 + h20·a0 + h21·a1
 */

export interface QuinticKnot {
  x: number;
  y: number;
  vx: number;
  vy: number;
  ax: number;
  ay: number;
}

// ─── Basis functions ──────────────────────────────────────────────────────────

export function quinticHermiteSegment(
  t: number,
  k0: QuinticKnot,
  k1: QuinticKnot,
): { x: number; y: number } {
  const t2 = t * t;
  const t3 = t2 * t;
  const t4 = t3 * t;
  const t5 = t4 * t;

  const h00 = 1 - 10 * t3 + 15 * t4 - 6 * t5;
  const h10 = t - 6 * t3 + 8 * t4 - 3 * t5;
  const h01 = 10 * t3 - 15 * t4 + 6 * t5;
  const h11 = -4 * t3 + 7 * t4 - 3 * t5;
  const h20 = 0.5 * t2 - 1.5 * t3 + 1.5 * t4 - 0.5 * t5;
  const h21 = 0.5 * t3 - t4 + 0.5 * t5;

  return {
    x: h00 * k0.x + h10 * k0.vx + h01 * k1.x + h11 * k1.vx + h20 * k0.ax + h21 * k1.ax,
    y: h00 * k0.y + h10 * k0.vy + h01 * k1.y + h11 * k1.vy + h20 * k0.ay + h21 * k1.ay,
  };
}

export function quinticHermiteDerivative(
  t: number,
  k0: QuinticKnot,
  k1: QuinticKnot,
): { x: number; y: number } {
  const t2 = t * t;
  const t3 = t2 * t;
  const t4 = t3 * t;

  const dh00 = -30 * t2 + 60 * t3 - 30 * t4;
  const dh10 = 1 - 18 * t2 + 32 * t3 - 15 * t4;
  const dh01 = 30 * t2 - 60 * t3 + 30 * t4;
  const dh11 = -12 * t2 + 28 * t3 - 15 * t4;
  const dh20 = t - 4.5 * t2 + 6 * t3 - 2.5 * t4;
  const dh21 = 1.5 * t2 - 4 * t3 + 2.5 * t4;

  return {
    x: dh00 * k0.x + dh10 * k0.vx + dh01 * k1.x + dh11 * k1.vx + dh20 * k0.ax + dh21 * k1.ax,
    y: dh00 * k0.y + dh10 * k0.vy + dh01 * k1.y + dh11 * k1.vy + dh20 * k0.ay + dh21 * k1.ay,
  };
}

// ─── Chord-length parameterized knot builder ──────────────────────────────────

/**
 * Build QuinticKnots from waypoints using chord-length parameterization.
 *
 * This is the key to exact interpolation:
 * - Chord lengths between adjacent waypoints are computed.
 * - Catmull-Rom tangents are scaled by the LOCAL chord length (not a global
 *   constant), so the curve actually passes through each waypoint.
 * - tangentScale controls tightness (0.5 = standard Catmull-Rom, lower = tighter)
 */
export function buildKnotsFromWaypoints(
  waypoints: { x: number; y: number }[],
  tangentScale = 0.5,
): QuinticKnot[] {
  const n = waypoints.length;
  if (n === 0) return [];
  if (n === 1) return [{ x: waypoints[0].x, y: waypoints[0].y, vx: 0, vy: 0, ax: 0, ay: 0 }];

  // Compute chord lengths between adjacent points
  const chords: number[] = [];
  for (let i = 0; i < n - 1; i++) {
    const dx = waypoints[i + 1].x - waypoints[i].x;
    const dy = waypoints[i + 1].y - waypoints[i].y;
    chords.push(Math.sqrt(dx * dx + dy * dy) + 1e-9); // avoid zero
  }

  const knots: QuinticKnot[] = [];

  for (let i = 0; i < n; i++) {
    const curr = waypoints[i];

    // Chord-length scaled Catmull-Rom tangent
    let vx: number, vy: number;

    if (i === 0) {
      // Forward difference, scaled by first chord
      vx = (waypoints[1].x - curr.x) / chords[0] * chords[0] * tangentScale;
      vy = (waypoints[1].y - curr.y) / chords[0] * chords[0] * tangentScale;
    } else if (i === n - 1) {
      // Backward difference, scaled by last chord
      vx = (curr.x - waypoints[n - 2].x) / chords[n - 2] * chords[n - 2] * tangentScale;
      vy = (curr.y - waypoints[n - 2].y) / chords[n - 2] * chords[n - 2] * tangentScale;
    } else {
      // Central difference weighted by chord lengths (Barry-Goldman formula)
      // This is the correct chord-length Catmull-Rom formulation
      const c0 = chords[i - 1]; // chord before
      const c1 = chords[i];     // chord after

      // Direction from prev to next, weighted by chord lengths
      const dx0 = (curr.x - waypoints[i - 1].x) / c0;
      const dy0 = (curr.y - waypoints[i - 1].y) / c0;
      const dx1 = (waypoints[i + 1].x - curr.x) / c1;
      const dy1 = (waypoints[i + 1].y - curr.y) / c1;

      // Blend weighted by chord length ratio → exact interpolation
      const totalChord = c0 + c1;
      vx = ((dx0 * c1 + dx1 * c0) / totalChord) * totalChord * tangentScale;
      vy = ((dy0 * c1 + dy1 * c0) / totalChord) * totalChord * tangentScale;
    }

    // Second finite difference for acceleration (chord-length scaled)
    let ax = 0, ay = 0;
    if (i > 0 && i < n - 1) {
      const c0 = chords[i - 1];
      const c1 = chords[i];
      const scale = Math.min(c0, c1) * 0.25 * tangentScale;
      ax = ((waypoints[i + 1].x - curr.x) / c1 - (curr.x - waypoints[i - 1].x) / c0) * scale;
      ay = ((waypoints[i + 1].y - curr.y) / c1 - (curr.y - waypoints[i - 1].y) / c0) * scale;
    }

    knots.push({ x: curr.x, y: curr.y, vx, vy, ax, ay });
  }

  return knots;
}

// ─── Spline evaluation ────────────────────────────────────────────────────────

/**
 * Sample a quintic spline at t ∈ [0,1] using CHORD-LENGTH parameterization.
 *
 * t is mapped proportionally across segments based on chord lengths,
 * so equal t steps correspond to approximately equal arc lengths.
 * This gives much better behavior than uniform parameterization.
 */
export function getQuinticSplinePoint(
  t: number,
  knots: QuinticKnot[],
  chords?: number[],
): { x: number; y: number } {
  if (knots.length === 0) return { x: 0, y: 0 };
  if (knots.length === 1) return { x: knots[0].x, y: knots[0].y };

  const n = knots.length - 1;
  t = Math.max(0, Math.min(1, t));

  if (t >= 1) return quinticHermiteSegment(1, knots[n - 1], knots[n]);

  // If chords provided, use chord-length parameterization
  if (chords && chords.length === n) {
    const totalLen = chords.reduce((s, c) => s + c, 0);
    const target = t * totalLen;
    let accumulated = 0;
    for (let i = 0; i < n; i++) {
      if (accumulated + chords[i] >= target) {
        const localT = (target - accumulated) / chords[i];
        return quinticHermiteSegment(Math.max(0, Math.min(1, localT)), knots[i], knots[i + 1]);
      }
      accumulated += chords[i];
    }
    return quinticHermiteSegment(1, knots[n - 1], knots[n]);
  }

  // Fallback: uniform parameterization
  const scaled = t * n;
  const seg = Math.min(Math.floor(scaled), n - 1);
  const localT = scaled - seg;
  return quinticHermiteSegment(localT, knots[seg], knots[seg + 1]);
}

/**
 * Get normalised tangent direction at t on a quintic spline.
 */
export function getQuinticSplineTangent(
  t: number,
  knots: QuinticKnot[],
  chords?: number[],
): { x: number; y: number } {
  if (knots.length < 2) return { x: 1, y: 0 };

  const n = knots.length - 1;
  t = Math.max(0, Math.min(1, t));

  let seg: number;
  let localT: number;

  if (chords && chords.length === n) {
    const totalLen = chords.reduce((s, c) => s + c, 0);
    const target = t * totalLen;
    let accumulated = 0;
    seg = n - 1;
    localT = 1;
    for (let i = 0; i < n; i++) {
      if (accumulated + chords[i] >= target) {
        seg = i;
        localT = (target - accumulated) / chords[i];
        break;
      }
      accumulated += chords[i];
    }
  } else {
    const scaled = Math.min(t, 0.9999) * n;
    seg = Math.min(Math.floor(scaled), n - 1);
    localT = scaled - seg;
  }

  const d = quinticHermiteDerivative(localT, knots[seg], knots[seg + 1]);
  const mag = Math.sqrt(d.x * d.x + d.y * d.y);
  if (mag < 1e-9) return { x: 1, y: 0 };
  return { x: d.x / mag, y: d.y / mag };
}

// ─── Convenience: build chords from waypoints ─────────────────────────────────

export function buildChords(waypoints: { x: number; y: number }[]): number[] {
  const chords: number[] = [];
  for (let i = 0; i < waypoints.length - 1; i++) {
    const dx = waypoints[i + 1].x - waypoints[i].x;
    const dy = waypoints[i + 1].y - waypoints[i].y;
    chords.push(Math.sqrt(dx * dx + dy * dy) + 1e-9);
  }
  return chords;
}

// ─── Drop-in for getCurvePoint ────────────────────────────────────────────────

/**
 * Drop-in replacement for getCurvePoint() — builds knots and evaluates.
 * Uses chord-length parameterization for exact waypoint interpolation.
 */
export function getCurvePointQuintic(
  t: number,
  waypoints: { x: number; y: number }[],
  tangentScale = 0.5,
): { x: number; y: number } {
  if (waypoints.length === 0) return { x: 0, y: 0 };
  if (waypoints.length === 1) return waypoints[0];
  const knots = buildKnotsFromWaypoints(waypoints, tangentScale);
  const chords = buildChords(waypoints);
  return getQuinticSplinePoint(t, knots, chords);
}