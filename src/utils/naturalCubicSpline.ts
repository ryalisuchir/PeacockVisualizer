/**
 * naturalCubicSpline.ts
 *
 * Natural cubic spline with chord-length parameterization.
 *
 * Guarantees exact interpolation through every waypoint by solving the
 * standard tridiagonal system for C2 continuity. The "natural" boundary
 * condition sets the second derivative to zero at both ends.
 *
 * Chord-length parameterization distributes t proportionally to arc length
 * so the curve doesn't bunch up around closely-spaced points.
 */

export interface CubicSegment {
  // Cubic polynomial coefficients: p(t) = a + b*t + c*t² + d*t³  (t in [0,1])
  ax: number; bx: number; cx: number; dx: number;
  ay: number; by: number; cy: number; dy: number;
  chord: number; // chord length of this segment (used for parameterization)
}

// ─── Solver ───────────────────────────────────────────────────────────────────

/**
 * Solve a tridiagonal system Ax = b using Thomas algorithm.
 * a = sub-diagonal, b = main diagonal, c = super-diagonal, d = RHS.
 * All arrays length n; a[0] and c[n-1] are unused.
 */
function thomasSolve(
  a: number[], b: number[], c: number[], d: number[],
): number[] {
  const n = d.length;
  const cp = new Array(n).fill(0);
  const dp = new Array(n).fill(0);
  const x  = new Array(n).fill(0);

  cp[0] = c[0] / b[0];
  dp[0] = d[0] / b[0];

  for (let i = 1; i < n; i++) {
    const m = b[i] - a[i] * cp[i - 1];
    cp[i] = c[i] / m;
    dp[i] = (d[i] - a[i] * dp[i - 1]) / m;
  }

  x[n - 1] = dp[n - 1];
  for (let i = n - 2; i >= 0; i--) {
    x[i] = dp[i] - cp[i] * x[i + 1];
  }
  return x;
}

/**
 * Build natural cubic spline segments from waypoints.
 * Returns (n-1) segments for n waypoints.
 *
 * Uses chord-length parameterization: each segment's local t ∈ [0,1]
 * is scaled by its chord length, so global t ∈ [0,1] maps proportionally
 * to arc length.
 */
export function buildNaturalCubicSpline(
  waypoints: { x: number; y: number }[],
): CubicSegment[] {
  const n = waypoints.length;
  if (n < 2) return [];

  // Compute chord lengths (h[i] = distance from point i to i+1)
  const h: number[] = [];
  for (let i = 0; i < n - 1; i++) {
    const dx = waypoints[i + 1].x - waypoints[i].x;
    const dy = waypoints[i + 1].y - waypoints[i].y;
    h.push(Math.max(Math.sqrt(dx * dx + dy * dy), 1e-9));
  }

  // Solve for second derivatives (one system per axis, same matrix)
  // Natural BC: M[0] = M[n-1] = 0
  // Interior: h[i-1]*M[i-1] + 2*(h[i-1]+h[i])*M[i] + h[i]*M[i+1] = RHS
  const m = n - 2; // number of interior knots
  if (m <= 0) {
    // Only 2 points: straight line
    const seg: CubicSegment = {
      ax: waypoints[0].x, bx: waypoints[1].x - waypoints[0].x, cx: 0, dx: 0,
      ay: waypoints[0].y, by: waypoints[1].y - waypoints[0].y, cy: 0, dy: 0,
      chord: h[0],
    };
    return [seg];
  }

  const solveAxis = (vals: number[]): number[] => {
    // Build tridiagonal for interior M values
    const a = new Array(m).fill(0); // sub-diagonal
    const b = new Array(m).fill(0); // main diagonal
    const c = new Array(m).fill(0); // super-diagonal
    const d = new Array(m).fill(0); // RHS

    for (let i = 0; i < m; i++) {
      const ki = i + 1; // interior knot index
      b[i] = 2 * (h[ki - 1] + h[ki]);
      if (i > 0)     a[i] = h[ki - 1];
      if (i < m - 1) c[i] = h[ki];
      d[i] = 6 * (
        (vals[ki + 1] - vals[ki]) / h[ki] -
        (vals[ki] - vals[ki - 1]) / h[ki - 1]
      );
    }

    const interior = thomasSolve(a, b, c, d);

    // Full M array with natural BC
    const M = new Array(n).fill(0);
    for (let i = 0; i < m; i++) M[i + 1] = interior[i];
    return M;
  };

  const xs = waypoints.map(p => p.x);
  const ys = waypoints.map(p => p.y);
  const Mx = solveAxis(xs);
  const My = solveAxis(ys);

  // Build segments
  const segments: CubicSegment[] = [];
  for (let i = 0; i < n - 1; i++) {
    const hi = h[i];
    // Standard cubic spline coefficients (local t ∈ [0, hi])
    // Re-parameterize to local t ∈ [0, 1] by substituting u = t * hi
    const ax = waypoints[i].x;
    const bx = (waypoints[i + 1].x - waypoints[i].x) / hi
             - hi * (2 * Mx[i] + Mx[i + 1]) / 6;
    const cx_raw = Mx[i] / 2;
    const dx_raw = (Mx[i + 1] - Mx[i]) / (6 * hi);

    const ay = waypoints[i].y;
    const by = (waypoints[i + 1].y - waypoints[i].y) / hi
             - hi * (2 * My[i] + My[i + 1]) / 6;
    const cy_raw = My[i] / 2;
    const dy_raw = (My[i + 1] - My[i]) / (6 * hi);

    // Convert from t ∈ [0, hi] to u ∈ [0, 1]: substitute t = u * hi
    segments.push({
      ax,
      bx: bx * hi,
      cx: cx_raw * hi * hi,
      dx: dx_raw * hi * hi * hi,
      ay,
      by: by * hi,
      cy: cy_raw * hi * hi,
      dy: dy_raw * hi * hi * hi,
      chord: hi,
    });
  }

  return segments;
}

// ─── Evaluation ───────────────────────────────────────────────────────────────

/** Evaluate a segment at local t ∈ [0, 1] */
function evalSegment(seg: CubicSegment, t: number): { x: number; y: number } {
  const t2 = t * t, t3 = t2 * t;
  return {
    x: seg.ax + seg.bx * t + seg.cx * t2 + seg.dx * t3,
    y: seg.ay + seg.by * t + seg.cy * t2 + seg.dy * t3,
  };
}

/** Evaluate derivative of a segment at local t ∈ [0, 1] */
function evalSegmentDeriv(seg: CubicSegment, t: number): { x: number; y: number } {
  const t2 = t * t;
  return {
    x: seg.bx + 2 * seg.cx * t + 3 * seg.dx * t2,
    y: seg.by + 2 * seg.cy * t + 3 * seg.dy * t2,
  };
}

/** Convert global t ∈ [0,1] to (segmentIndex, localT) using chord-length mapping */
function globalToLocal(
  t: number,
  segments: CubicSegment[],
): { seg: CubicSegment; localT: number } {
  t = Math.max(0, Math.min(1, t));
  const totalChord = segments.reduce((s, seg) => s + seg.chord, 0);
  const target = t * totalChord;

  let accumulated = 0;
  for (let i = 0; i < segments.length; i++) {
    const next = accumulated + segments[i].chord;
    if (next >= target || i === segments.length - 1) {
      const localT = segments[i].chord > 1e-9
        ? Math.max(0, Math.min(1, (target - accumulated) / segments[i].chord))
        : 0;
      return { seg: segments[i], localT };
    }
    accumulated = next;
  }
  return { seg: segments[segments.length - 1], localT: 1 };
}

/**
 * Sample the natural cubic spline at global t ∈ [0,1].
 * Chord-length parameterized — passes exactly through all waypoints.
 */
export function getCubicSplinePoint(
  t: number,
  segments: CubicSegment[],
): { x: number; y: number } {
  if (segments.length === 0) return { x: 0, y: 0 };
  const { seg, localT } = globalToLocal(t, segments);
  return evalSegment(seg, localT);
}

/**
 * Get normalised tangent direction at global t ∈ [0,1].
 */
export function getCubicSplineTangent(
  t: number,
  segments: CubicSegment[],
): { x: number; y: number } {
  if (segments.length === 0) return { x: 1, y: 0 };
  const { seg, localT } = globalToLocal(t, segments);
  const d = evalSegmentDeriv(seg, localT);
  const mag = Math.sqrt(d.x * d.x + d.y * d.y);
  if (mag < 1e-9) return { x: 1, y: 0 };
  return { x: d.x / mag, y: d.y / mag };
}

/**
 * Drop-in for getCurvePoint / getCurvePointQuintic.
 * Builds a natural cubic spline and evaluates at t.
 */
export function getCurvePointCubic(
  t: number,
  waypoints: { x: number; y: number }[],
): { x: number; y: number } {
  if (waypoints.length === 0) return { x: 0, y: 0 };
  if (waypoints.length === 1) return waypoints[0];
  if (waypoints.length === 2) {
    return {
      x: waypoints[0].x + (waypoints[1].x - waypoints[0].x) * t,
      y: waypoints[0].y + (waypoints[1].y - waypoints[0].y) * t,
    };
  }
  const segs = buildNaturalCubicSpline(waypoints);
  return getCubicSplinePoint(t, segs);
}