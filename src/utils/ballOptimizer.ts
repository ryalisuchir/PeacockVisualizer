/**
 * ballOptimizer.ts
 *
 * COORDINATE SYSTEM: x right, y up, (0,0) bottom-left, 144"x144" field.
 *
 * KEY RULES:
 *  - Balls inside or within ~robot_half_size of an obstacle are IGNORED.
 *  - All path segments checked against walls + obstacle polygons.
 *  - TIGHT TRIANGLE: if all 3 balls fit within a 2.5×robotWidth bounding box,
 *    the robot drives straight through the centroid in one pass (no control pts).
 *  - COLLINEAR: if the 3 robot-center positions deviate < 8" from a straight
 *    line, emit a single straight segment with no control points.
 *  - Triangle formation (1 outlier + 2 wall balls): single smooth Bézier
 *    through outlier RC → firstWallRC, then straight strafe.
 *  - Wall sweep: approach arrives at wallFacingDeg (heading reset), then
 *    straight strafe opening toward balls (sweepSign * 35°).
 *  - Middle-ball fix: cp2 placement uses perpendicular swing distance so the
 *    curve fully commits to the arrival heading before reaching each ball.
 */

import type { Point, Line, SequenceItem } from "../types";
import { getRandomColor } from "./draw";

// ─── Public types ─────────────────────────────────────────────────────────────

export interface BallPosition { id: string; x: number; y: number; }

export interface ObstaclePolygon {
  vertices: { x: number; y: number }[];
}

export interface OptimizerWeights {
  wIntake: number; wHeading: number; wCurvature: number; wCentripetal: number;
}
export const DEFAULT_OPTIMIZER_WEIGHTS: OptimizerWeights = {
  wIntake: 1.0, wHeading: 0.6, wCurvature: 0.4, wCentripetal: 0.3,
};

export interface OptimizerSettings {
  wallMargin: number; robotWidth: number; robotHeight: number;
  fieldSize: number; ballRadius: number; intakeOffset: number; tangentScale: number;
  obstacles?: ObstaclePolygon[];
}
export const DEFAULT_OPTIMIZER_SETTINGS: OptimizerSettings = {
  wallMargin: 8, robotWidth: 18, robotHeight: 18, fieldSize: 144,
  ballRadius: 2.5, intakeOffset: 0, tangentScale: 0.5, obstacles: [],
};

export interface OptimalPathResult {
  startPoint: Point; lines: Line[]; sequence: SequenceItem[];
}

// ─── Geometry helpers ─────────────────────────────────────────────────────────

function clamp(v: number, lo: number, hi: number) { return Math.max(lo, Math.min(hi, v)); }
function clampToField(p: { x: number; y: number }, rw: number, rh: number, fs: number) {
  return { x: clamp(p.x, rw/2, fs-rw/2), y: clamp(p.y, rh/2, fs-rh/2) };
}
function vecLen(dx: number, dy: number) { return Math.sqrt(dx*dx + dy*dy) + 1e-9; }
function dist(a: { x: number; y: number }, b: { x: number; y: number }) {
  return Math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2);
}
function angleDeg(from: { x:number; y:number }, to: { x:number; y:number }): number {
  return Math.atan2(to.y - from.y, to.x - from.x) * 180 / Math.PI;
}

type Wall = "left" | "right" | "bottom" | "top";

function nearestWall(b: BallPosition, margin: number, fs: number): Wall | null {
  const c: { wall: Wall; d: number }[] = [
    { wall: "left", d: b.x }, { wall: "right", d: fs-b.x },
    { wall: "bottom", d: b.y }, { wall: "top", d: fs-b.y },
  ];
  const m = c.reduce((a, x) => x.d < a.d ? x : a);
  return m.d < margin ? m.wall : null;
}
function wallParallelAxis(w: Wall): "x" | "y" { return w === "left" || w === "right" ? "y" : "x"; }
function ballDistFromWall(b: BallPosition, w: Wall, fs: number): number {
  switch (w) { case "bottom": return b.y; case "top": return fs-b.y; case "right": return fs-b.x; case "left": return b.x; }
}
function getWallRC(b: BallPosition, w: Wall, io: number, rw: number, rh: number, fs: number) {
  // Robot center is positioned so the intake face (at io from center) hits the ball.
  // The perpendicular axis (into the wall) is constrained to io from the wall.
  // The parallel axis (along the wall) follows the ball's position exactly,
  // so the strafe path is diagonal rather than a fixed vertical/horizontal line.
  switch (w) {
    case "bottom": return clampToField({ x: b.x, y: io      }, rw, rh, fs);
    case "top":    return clampToField({ x: b.x, y: fs-io   }, rw, rh, fs);
    case "right":  return clampToField({ x: fs-io, y: b.y   }, rw, rh, fs);
    case "left":   return clampToField({ x: io,    y: b.y   }, rw, rh, fs);
  }
}
function getWallStrafeStopRC(
  b: BallPosition,
  w: Wall,
  io: number,
  rw: number,
  rh: number,
  fs: number,
  sweepSign: number,
  stopDist: number,
) {
  const base = getWallRC(b, w, io, rw, rh, fs);
  const axis = wallParallelAxis(w);
  if (axis === "y") {
    return clampToField({ x: base.x, y: base.y - sweepSign * stopDist }, rw, rh, fs);
  }
  return clampToField({ x: base.x - sweepSign * stopDist, y: base.y }, rw, rh, fs);
}
function applyWallInset(
  rc: { x:number; y:number },
  w: Wall,
  inset: number,
  rw: number,
  rh: number,
  fs: number,
) {
  if (inset <= 0) return rc;
  switch (w) {
    case "right": return clampToField({ x: rc.x - inset, y: rc.y }, rw, rh, fs);
    case "left":  return clampToField({ x: rc.x + inset, y: rc.y }, rw, rh, fs);
    case "top":   return clampToField({ x: rc.x, y: rc.y - inset }, rw, rh, fs);
    case "bottom":return clampToField({ x: rc.x, y: rc.y + inset }, rw, rh, fs);
  }
}
function wallFacingDeg(w: Wall): number {
  switch (w) { case "right": return 0; case "left": return 180; case "top": return 90; case "bottom": return -90; }
}
function sortForSweep(balls: BallPosition[], w: Wall, startPos: { x: number; y: number }, fs: number) {
  // Sort balls along the wall-parallel axis (Y for left/right walls, X for top/bottom).
  const byParallel = wallParallelAxis(w) === "y"
    ? [...balls].sort((a, b) => a.y - b.y)
    : [...balls].sort((a, b) => a.x - b.x);

  const first = byParallel[0], last = byParallel[byParallel.length - 1];
  const firstWallDist = ballDistFromWall(first as BallPosition, w, fs);
  const lastWallDist  = ballDistFromWall(last as BallPosition, w, fs);
  const distDiff = Math.abs(firstWallDist - lastWallDist);

  let startFromFirst: boolean;
  if (distDiff > 3) {
    // One endpoint is clearly further from the wall — start there so the
    // intake is already opening toward it on arrival.
    startFromFirst = firstWallDist > lastWallDist;
  } else {
    // Equal wall distances (e.g. after promoting middle ball to outlier):
    // start from whichever endpoint the robot is closer to.
    startFromFirst = dist(startPos, first) <= dist(startPos, last);
  }

  return startFromFirst
    ? { sorted: byParallel,                  sweepSign:  1 }
    : { sorted: [...byParallel].reverse(),   sweepSign: -1 };
}

// ─── Obstacle helpers ─────────────────────────────────────────────────────────

function pointInPolygon(px: number, py: number, verts: { x: number; y: number }[]): boolean {
  let inside = false;
  for (let i = 0, j = verts.length-1; i < verts.length; j = i++) {
    const xi = verts[i].x, yi = verts[i].y, xj = verts[j].x, yj = verts[j].y;
    if ((yi > py) !== (yj > py) && px < ((xj-xi)*(py-yi))/(yj-yi)+xi) inside = !inside;
  }
  return inside;
}

function minDistToPolygon(px: number, py: number, verts: { x: number; y: number }[]): number {
  let minD = Infinity;
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i], b = verts[(i+1)%verts.length];
    const dx = b.x-a.x, dy = b.y-a.y, len2 = dx*dx+dy*dy;
    const t = len2 > 0 ? clamp(((px-a.x)*dx+(py-a.y)*dy)/len2, 0, 1) : 0;
    minD = Math.min(minD, Math.sqrt((px-a.x-t*dx)**2+(py-a.y-t*dy)**2));
  }
  return minD;
}

function robotHitsObstacle(cx: number, cy: number, hw: number, hh: number,
  obstacles: ObstaclePolygon[], margin = 2): boolean {
  const pts = [
    {x:cx,    y:cy   }, {x:cx-hw, y:cy-hh}, {x:cx+hw, y:cy-hh},
    {x:cx+hw, y:cy+hh}, {x:cx-hw, y:cy+hh},
  ];
  for (const obs of obstacles) {
    for (const pt of pts) {
      if (pointInPolygon(pt.x, pt.y, obs.vertices)) return true;
      if (minDistToPolygon(pt.x, pt.y, obs.vertices) < margin) return true;
    }
  }
  return false;
}

function filterReachableBalls(
  balls: BallPosition[], io: number, rw: number, rh: number, fs: number,
  obstacles: ObstaclePolygon[],
): BallPosition[] {
  if (obstacles.length === 0) return balls;
  const hw = rw/2;
  return balls.filter(b => {
    for (const obs of obstacles) {
      if (pointInPolygon(b.x, b.y, obs.vertices)) return false;
      if (minDistToPolygon(b.x, b.y, obs.vertices) < hw + 2) return false;
    }
    return true;
  });
}

function sampleBezier(t: number,
  p0: {x:number;y:number}, cp1: {x:number;y:number},
  cp2: {x:number;y:number}, p3: {x:number;y:number}): {x:number;y:number} {
  const mt=1-t, mt2=mt*mt, t2=t*t, mt3=mt2*mt, t3=t2*t;
  return { x: mt3*p0.x+3*mt2*t*cp1.x+3*mt*t2*cp2.x+t3*p3.x,
           y: mt3*p0.y+3*mt2*t*cp1.y+3*mt*t2*cp2.y+t3*p3.y };
}

function pathHitsObstruction(
  p0: {x:number;y:number}, cp1: {x:number;y:number},
  cp2: {x:number;y:number}, p3: {x:number;y:number},
  rw: number, rh: number, fs: number, obstacles: ObstaclePolygon[], samples=24): boolean {
  const hw=rw/2, hh=rh/2;
  for (let i=0; i<=samples; i++) {
    const p = sampleBezier(i/samples, p0, cp1, cp2, p3);
    if (p.x-hw<0||p.x+hw>fs||p.y-hh<0||p.y+hh>fs) return true;
    if (robotHitsObstacle(p.x, p.y, hw, hh, obstacles)) return true;
  }
  return false;
}

function findDetourMidpoint(
  from: {x:number;y:number}, to: {x:number;y:number},
  rw: number, rh: number, fs: number, obstacles: ObstaclePolygon[],
): {x:number;y:number}|null {
  const mx=(from.x+to.x)/2, my=(from.y+to.y)/2;
  const dx=to.x-from.x, dy=to.y-from.y, d=vecLen(dx,dy);
  const px=-dy/d, py=dx/d;
  for (const offset of [20, 32, 48, 64]) {
    for (const sign of [1,-1]) {
      const mid = { x: clamp(mx+px*offset*sign, rw/2, fs-rw/2), y: clamp(my+py*offset*sign, rh/2, fs-rh/2) };
      const t = offset*0.4;
      const m1ux=(mid.x-from.x)/vecLen(mid.x-from.x,mid.y-from.y);
      const m1uy=(mid.y-from.y)/vecLen(mid.x-from.x,mid.y-from.y);
      const m2ux=(to.x-mid.x)/vecLen(to.x-mid.x,to.y-mid.y);
      const m2uy=(to.y-mid.y)/vecLen(to.x-mid.x,to.y-mid.y);
      if (!pathHitsObstruction(from,{x:from.x+m1ux*t,y:from.y+m1uy*t},{x:mid.x-m1ux*t,y:mid.y-m1uy*t},mid,rw,rh,fs,obstacles) &&
          !pathHitsObstruction(mid, {x:mid.x+m2ux*t, y:mid.y+m2uy*t}, {x:to.x-m2ux*t,  y:to.y-m2uy*t}, to, rw,rh,fs,obstacles)) {
        return mid;
      }
    }
  }
  return null;
}

// ─── Permutations ─────────────────────────────────────────────────────────────

function permutations<T>(arr: T[]): T[][] {
  if (arr.length <= 1) return [arr];
  const result: T[][] = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = [...arr.slice(0,i), ...arr.slice(i+1)];
    for (const p of permutations(rest)) result.push([arr[i], ...p]);
  }
  return result;
}

function getRobotCenter(from: {x:number;y:number}, ball: BallPosition,
  io: number, rw: number, rh: number, fs: number) {
  const dx=ball.x-from.x, dy=ball.y-from.y, d=vecLen(dx,dy);
  return clampToField({x:ball.x-(dx/d)*io, y:ball.y-(dy/d)*io}, rw, rh, fs);
}

// ─── NEW: Tight-cluster & collinearity checks ─────────────────────────────────

/**
 * Returns true if all balls fit within a bounding box small enough that
 * a single straight pass through the centroid will intake all of them.
 * Threshold: max spread ≤ robotWidth × 2.0 in each axis.
 */
/**
 * Returns true if ALL balls are within 2" gap of each other (not just touching).
 * center-to-center ≤ ballDiameter + 2" = 7"
 */
function isTightCluster(balls: BallPosition[], ballRadius: number, robotWidth = 18): boolean {
  if (balls.length < 2) return false;
  // Cluster = all balls within 3" gap of each other (center-to-center ≤ diameter + 3" = 8")
  // AND not arranged in a line (collinear balls should use normal path, not cluster intake).
  const threshold = ballRadius * 2 + 3; // ~8"
  for (let i = 0; i < balls.length; i++) {
    for (let j = i + 1; j < balls.length; j++) {
      if (dist(balls[i], balls[j]) > threshold) return false;
    }
  }
  // Exclude collinear arrangements — if all balls are on a line, normal path handles it better
  if (balls.length >= 3) {
    const start = balls[0], end = balls[balls.length - 1];
    const dx = end.x - start.x, dy = end.y - start.y;
    const len = vecLen(dx, dy);
    const ux = dx/len, uy = dy/len;
    for (let i = 1; i < balls.length - 1; i++) {
      const px = balls[i].x - start.x, py = balls[i].y - start.y;
      const perp = Math.abs(px * uy - py * ux);
      if (perp > 3) return true; // non-collinear → is a cluster
    }
    return false; // all collinear → not a cluster
  }
  return true;
}

/**
 * Returns true if balls are "medium close" — spread fits within robotWidth * 1.5
 * but they're NOT a tight cluster. Triggers spline-through-all approach.
 */
function isMediumCluster(balls: BallPosition[], robotWidth: number, ballRadius: number): boolean {
  if (isTightCluster(balls, ballRadius, robotWidth)) return false;
  // Medium cluster = all balls within 3" of each other but not collinear.
  // Same 8" pairwise threshold — this is just the non-tight version for 2 balls.
  // (For 3 balls, isTightCluster already handles the collinear check above.)
  const threshold = ballRadius * 2 + 3; // ~8"
  for (let i = 0; i < balls.length; i++) {
    for (let j = i + 1; j < balls.length; j++) {
      if (dist(balls[i], balls[j]) > threshold) return false;
    }
  }
  return true;
}

/**
 * Triangle pattern: left + top + bottom, with top/bottom roughly aligned vertically.
 * This matches the "left + top then strafe to bottom" intake pattern.
 */
function detectTriangleStrafePattern(
  balls: BallPosition[],
  robotWidth: number,
): { left: BallPosition; top: BallPosition; bottom: BallPosition } | null {
  if (balls.length !== 3) return null;
  const left = [...balls].sort((a, b) => a.x - b.x)[0];
  const top = [...balls].sort((a, b) => b.y - a.y)[0];
  const bottom = [...balls].sort((a, b) => a.y - b.y)[0];

  if (left.id === top.id || left.id === bottom.id || top.id === bottom.id) return null;

  // Non-collinear check via triangle area
  const area2 = Math.abs(
    (top.x - left.x) * (bottom.y - left.y) - (top.y - left.y) * (bottom.x - left.x),
  );
  if (area2 < robotWidth * 4) return null; // too flat

  // Require top/bottom to be vertically aligned enough to make a clean strafe
  const alignTol = Math.max(6, robotWidth * 0.35);
  if (Math.abs(top.x - bottom.x) > alignTol) return null;

  // Ensure left is meaningfully left of the top/bottom column
  if (left.x > Math.min(top.x, bottom.x) - alignTol) return null;

  return { left, top, bottom };
}

/**
 * Straight vertical line pattern: balls are roughly collinear with very small X spread.
 * Intended for "strafe down through all balls" behavior.
 */
function detectVerticalStrafePattern(
  balls: BallPosition[],
  robotWidth: number,
): { top: BallPosition; bottom: BallPosition } | null {
  if (balls.length !== 3) return null;
  const xs = balls.map(b => b.x);
  const ys = balls.map(b => b.y);
  const xSpread = Math.max(...xs) - Math.min(...xs);
  const ySpread = Math.max(...ys) - Math.min(...ys);
  if (ySpread < robotWidth * 0.8) return null; // not a tall line
  if (xSpread > Math.max(6, robotWidth * 0.35)) return null; // too wide to be a strafe line

  const sortedByY = [...balls].sort((a, b) => b.y - a.y);
  return { top: sortedByY[0], bottom: sortedByY[2] };
}

/**
 * Two balls on the left column, one ball on the right (outlier).
 * Use: approach top-left with open heading, strafe down to bottom-left, then curve to right.
 */
function detectTwoLeftOneRightPattern(
  balls: BallPosition[],
  robotWidth: number,
): { topLeft: BallPosition; bottomLeft: BallPosition; right: BallPosition } | null {
  if (balls.length !== 3) return null;
  const sortedByX = [...balls].sort((a, b) => a.x - b.x);
  const left1 = sortedByX[0];
  const left2 = sortedByX[1];
  const right = sortedByX[2];

  const leftSep = right.x - left2.x;
  if (leftSep < Math.max(8, robotWidth * 0.6)) return null; // not a clear right outlier

  const leftPairSpread = Math.abs(left1.x - left2.x);
  if (leftPairSpread > Math.max(6, robotWidth * 0.35)) return null; // left pair not aligned

  const topLeft = left1.y >= left2.y ? left1 : left2;
  const bottomLeft = left1.y < left2.y ? left1 : left2;
  if (Math.abs(topLeft.y - bottomLeft.y) < robotWidth * 0.6) return null; // not vertically separated

  return { topLeft, bottomLeft, right };
}

function detectTwoRightOneLeftPattern(
  balls: BallPosition[],
  robotWidth: number,
): { topRight: BallPosition; bottomRight: BallPosition; left: BallPosition } | null {
  if (balls.length !== 3) return null;
  // Ball 1 = top, Ball 2 = middle, Ball 3 = bottom (by Y)
  const sortedByY = [...balls].sort((a, b) => b.y - a.y);
  const top = sortedByY[0];
  const mid = sortedByY[1];
  const bottom = sortedByY[2];

  // "right right left" means top + mid are to the right of bottom
  const sep = Math.min(top.x - bottom.x, mid.x - bottom.x);
  if (sep < Math.max(8, robotWidth * 0.6)) return null;

  // Top and mid should be roughly aligned in X to form the right column
  const rightPairSpread = Math.abs(top.x - mid.x);
  if (rightPairSpread > Math.max(6, robotWidth * 0.35)) return null;

  return { topRight: top, bottomRight: mid, left: bottom };
}

function canStraightLineIntake(
  startPos: { x:number; y:number },
  balls: BallPosition[],
  rw: number,
  ballRadius: number,
): { dirUx: number; dirUy: number; farthest: BallPosition; maxPerp: number } | null {
  if (balls.length < 2) return null;
  const cen = centroid(balls);
  const dx = cen.x - startPos.x;
  const dy = cen.y - startPos.y;
  const d = vecLen(dx, dy);
  if (d < 1e-6) return null;
  const ux = dx / d, uy = dy / d;
  const intakeHalfWidth = (rw / 2) - 1; // intake is 1" inside the frame on each side
  const maxPerp = intakeHalfWidth + ballRadius + 0.5; // small buffer

  let farthest = balls[0];
  let farthestAlong = -Infinity;
  let maxPerpSeen = 0;
  for (const b of balls) {
    const vx = b.x - startPos.x;
    const vy = b.y - startPos.y;
    const along = vx * ux + vy * uy;
    const perp = Math.abs(vx * uy - vy * ux);
    if (perp > maxPerp) return null;
    if (perp > maxPerpSeen) maxPerpSeen = perp;
    if (along > farthestAlong) {
      farthestAlong = along;
      farthest = b;
    }
  }
  return { dirUx: ux, dirUy: uy, farthest, maxPerp: maxPerpSeen };
}

/**
 * Centroid of a set of balls.
 */
function centroid(balls: BallPosition[]): { x: number; y: number } {
  return {
    x: balls.reduce((s, b) => s + b.x, 0) / balls.length,
    y: balls.reduce((s, b) => s + b.y, 0) / balls.length,
  };
}

/**
 * Returns true if the robot-center positions for a given ordering are
 * approximately collinear (max perpendicular deviation < threshold).
 * Also returns true if there are only 2 points (always collinear).
 */
function areRobotCentersCollinear(
  rcs: { x: number; y: number }[],
  threshold = 8,
): boolean {
  if (rcs.length <= 2) return true;
  const start = rcs[0], end = rcs[rcs.length - 1];
  const dx = end.x - start.x, dy = end.y - start.y;
  const len = vecLen(dx, dy);
  const ux = dx / len, uy = dy / len;
  for (let i = 1; i < rcs.length - 1; i++) {
    const px = rcs[i].x - start.x, py = rcs[i].y - start.y;
    // Perpendicular distance = |cross product|
    const perp = Math.abs(px * uy - py * ux);
    if (perp > threshold) return false;
  }
  return true;
}

// ─── Segment builder helpers ──────────────────────────────────────────────────

function tangentialPoint(p: {x:number;y:number}): Point {
  return {x:p.x, y:p.y, heading:"tangential", reverse:false} as Point;
}
function linearPoint(p: {x:number;y:number}, startDeg: number, endDeg: number): Point {
  return {x:p.x, y:p.y, heading:"linear", startDeg, endDeg} as Point;
}
function makeStraight(ep: Point, name: string, color: string): Line {
  return { id:`straight-${Math.random().toString(36).slice(2,9)}`, name, endPoint:ep,
    controlPoints:[], color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"" };
}
function buildSafeStraightSegment(
  fromPos: {x:number;y:number},
  toPos: {x:number;y:number},
  startDeg: number,
  endDeg: number,
  name: string,
  color: string,
  rw: number, rh: number, fs: number, obstacles: ObstaclePolygon[],
): Line[] {
  const dx = toPos.x - fromPos.x;
  const dy = toPos.y - fromPos.y;
  const d = vecLen(dx, dy);
  if (d < 1e-6) {
    return [makeStraight(linearPoint(toPos, startDeg, endDeg), name, color)];
  }
  const ux = dx / d, uy = dy / d;
  const chord = d;
  const t = clamp(chord * 0.38, 6, 55);
  const cp1 = { x: fromPos.x + ux * t, y: fromPos.y + uy * t };
  const cp2 = { x: toPos.x - ux * t,   y: toPos.y - uy * t   };
  if (pathHitsObstruction(fromPos, cp1, cp2, toPos, rw, rh, fs, obstacles)) {
    if (name.startsWith("Wall Strafe")) {
      // Keep wall strafe as a single clean line (no detour segments)
      return [makeStraight(linearPoint(toPos, startDeg, endDeg), name, color)];
    }
  }
  return buildSafeSegment(
    fromPos, ux, uy, toPos, ux, uy,
    linearPoint(toPos, startDeg, endDeg),
    name, color, rw, rh, fs, obstacles,
  );
}

function buildSafeBezierSegment(
  fromPos: {x:number;y:number}, fromUx: number, fromUy: number,
  toPos: {x:number;y:number}, toUx: number, toUy: number,
  startDeg: number, endDeg: number,
  name: string, color: string,
  rw: number, rh: number, fs: number, obstacles: ObstaclePolygon[],
  tDep?: number, tArr?: number,
): Line[] {
  const chord = vecLen(toPos.x - fromPos.x, toPos.y - fromPos.y);
  const td = tDep ?? clamp(chord * 0.38, 6, 55);
  const ta = tArr ?? clamp(chord * 0.55, 8, 60);
  const cp1 = { x: fromPos.x + fromUx * td, y: fromPos.y + fromUy * td };
  const cp2 = { x: toPos.x - toUx * ta,    y: toPos.y - toUy * ta    };

  if (!pathHitsObstruction(fromPos, cp1, cp2, toPos, rw, rh, fs, obstacles)) {
    const seg: Line = {
      id: `curve-${Math.random().toString(36).slice(2,9)}`,
      name,
      endPoint: linearPoint(toPos, startDeg, endDeg),
      controlPoints: [cp1, cp2],
      color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"",
    };
    return [seg];
  }
  return buildSafeSegment(
    fromPos, fromUx, fromUy, toPos, toUx, toUy,
    linearPoint(toPos, startDeg, endDeg),
    name, color, rw, rh, fs, obstacles,
  );
}

function splineLine(
  from: { x: number; y: number },
  waypoints: { x: number; y: number }[],
  name: string,
  color: string,
): Line {
  // Natural cubic spline: from = start, waypoints = interior + end
  // Rendered as curveType "cubic" (passes through control points)
  const endWP = waypoints[waypoints.length - 1];
  const controlWPs = waypoints.slice(0, -1);
  return {
    id: `spline-${Math.random().toString(36).slice(2,9)}`,
    name,
    endPoint: tangentialPoint(endWP),
    controlPoints: controlWPs,
    color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"",
    curveType: "cubic",
  } as any;
}

function buildBezierChain(
  from: { x: number; y: number },
  waypoints: { x: number; y: number }[],
  name: string,
  color: string,
  rw: number, rh: number, fs: number, obstacles: ObstaclePolygon[],
  tangentialHeading = false,
): Line[] {
  const lines: Line[] = [];
  const pts = [from, ...waypoints];
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i];
    const b = pts[i + 1];
    const prev = i > 0 ? pts[i - 1] : a;
    const next = i + 2 < pts.length ? pts[i + 2] : b;

    const chord = vecLen(b.x - a.x, b.y - a.y);
    if (tangentialHeading) {
      // Catmull-style tangents for smooth long-distance curves
      const tanAx = b.x - prev.x, tanAy = b.y - prev.y;
      const tanBx = next.x - a.x, tanBy = next.y - a.y;
      const tanAD = vecLen(tanAx, tanAy);
      const tanBD = vecLen(tanBx, tanBy);
      const fromUx = tanAD > 1e-6 ? tanAx / tanAD : (b.x - a.x) / chord;
      const fromUy = tanAD > 1e-6 ? tanAy / tanAD : (b.y - a.y) / chord;
      const toUx = tanBD > 1e-6 ? tanBx / tanBD : (b.x - a.x) / chord;
      const toUy = tanBD > 1e-6 ? tanBy / tanBD : (b.y - a.y) / chord;
      const startDeg = Math.atan2(fromUy, fromUx) * 180 / Math.PI;
      const endDeg = Math.atan2(toUy, toUx) * 180 / Math.PI;
      const t = clamp(chord * 0.35, 6, 55);
      const segs = buildSafeBezierSegment(
        a, fromUx, fromUy,
        b, toUx, toUy,
        startDeg, endDeg,
        `${name} ${i + 1}`, color, rw, rh, fs, obstacles,
        t, t,
      );
      for (const seg of segs) {
        seg.endPoint = tangentialPoint(b);
      }
      lines.push(...segs);
      continue;
    }

    const fromDx = a.x - prev.x, fromDy = a.y - prev.y;
    const fromD = vecLen(fromDx, fromDy);
    const fromUx = fromD > 1e-6 ? fromDx / fromD : (b.x - a.x) / chord;
    const fromUy = fromD > 1e-6 ? fromDy / fromD : (b.y - a.y) / chord;

    const toDx = next.x - b.x, toDy = next.y - b.y;
    const toD = vecLen(toDx, toDy);
    const toUx = toD > 1e-6 ? toDx / toD : (b.x - a.x) / chord;
    const toUy = toD > 1e-6 ? toDy / toD : (b.y - a.y) / chord;

    const startDeg = Math.atan2(fromUy, fromUx) * 180 / Math.PI;
    const endDeg = Math.atan2(toUy, toUx) * 180 / Math.PI;
    const segs = buildSafeBezierSegment(
      a, fromUx, fromUy,
      b, toUx, toUy,
      startDeg, endDeg,
      `${name} ${i + 1}`, color, rw, rh, fs, obstacles,
    );
    if (tangentialHeading) {
      for (const seg of segs) {
        seg.endPoint = tangentialPoint(b);
      }
    }
    lines.push(...segs);
  }
  return lines;
}

/**
 * Build a safe Bézier or detour. Returns 1 or 2 Line segments.
 *
 * FIX for middle-ball issue:
 * tArr is now computed as a function of the PERPENDICULAR swing distance from
 * the straight fromPos→toPos line to the ball. A larger perpendicular offset
 * means the curve must swing wider, which requires cp2 to be pulled further
 * back so the robot is already aligned before it reaches the ball.
 *
 * Formula: tArr = max(chord * 0.55, perpOffset * 1.8, 12)
 * This ensures the curve is always fully committed to the arrival direction
 * at least tArr inches before the ball, which is enough for the intake face
 * to be correctly aligned at the moment of contact.
 */
function buildSafeSegment(
  fromPos: {x:number;y:number}, fromUx: number, fromUy: number,
  toPos: {x:number;y:number}, arrUx: number, arrUy: number,
  ep: Point, name: string, color: string,
  rw: number, rh: number, fs: number, obstacles: ObstaclePolygon[],
  ballPos?: {x:number;y:number},
): Line[] {
  const chord = vecLen(toPos.x-fromPos.x, toPos.y-fromPos.y);
  const tDep = clamp(chord*0.38, 6, 55);

  // Compute perpendicular distance of ball (or toPos) from the straight
  // fromPos→toPos line. Used to set arrival tension correctly.
  const refPoint = ballPos ?? toPos;
  const pathDx = toPos.x-fromPos.x, pathDy = toPos.y-fromPos.y;
  const pathLen = vecLen(pathDx, pathDy);
  const pathUx = pathDx/pathLen, pathUy = pathDy/pathLen;
  const toRefDx = refPoint.x-fromPos.x, toRefDy = refPoint.y-fromPos.y;
  const along = toRefDx*pathUx + toRefDy*pathUy;
  const latX = toRefDx - along*pathUx, latY = toRefDy - along*pathUy;
  const lateral = Math.sqrt(latX*latX + latY*latY);

  // tArr: cp2 is placed this far behind toPos along arrU.
  // Larger lateral offset → larger tArr → curve commits to arrival heading earlier.
  // This is the key fix for the "middle ball" problem: the Bézier was previously
  // under-committing (cp2 too close to toPos), causing the robot to only graze
  // the ball from the side rather than driving straight into it.
  const tArr = clamp(
    Math.max(chord * 0.55, lateral * 1.8, 12),
    12,
    chord * 1.6,
  );

  const cp1 = {x:fromPos.x+fromUx*tDep, y:fromPos.y+fromUy*tDep};
  const cp2 = {x:toPos.x-arrUx*tArr,    y:toPos.y-arrUy*tArr   };

  if (!pathHitsObstruction(fromPos, cp1, cp2, toPos, rw, rh, fs, obstacles)) {
    return [{ id:`curve-${Math.random().toString(36).slice(2,9)}`, name, endPoint:ep,
      controlPoints:[cp1,cp2], color, locked:false,
      waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"" }];
  }
  const mid = findDetourMidpoint(fromPos, toPos, rw, rh, fs, obstacles);
  if (mid) {
    const dDep = clamp(tDep*0.7, 4, 40);
    const dArr = clamp(tArr*0.7, 4, 40);
    const m1ux=(mid.x-fromPos.x)/vecLen(mid.x-fromPos.x,mid.y-fromPos.y);
    const m1uy=(mid.y-fromPos.y)/vecLen(mid.x-fromPos.x,mid.y-fromPos.y);
    const m2ux=(toPos.x-mid.x)/vecLen(toPos.x-mid.x,toPos.y-mid.y);
    const m2uy=(toPos.y-mid.y)/vecLen(toPos.x-mid.x,toPos.y-mid.y);
    return [
      { id:`det1-${Math.random().toString(36).slice(2,9)}`, name:`${name}a`,
        endPoint:tangentialPoint(mid),
        controlPoints:[{x:fromPos.x+fromUx*dDep,y:fromPos.y+fromUy*dDep},{x:mid.x-m1ux*dDep,y:mid.y-m1uy*dDep}],
        color, locked:false, waitBeforeMs:0,waitAfterMs:0,waitBeforeName:"",waitAfterName:"" },
      { id:`det2-${Math.random().toString(36).slice(2,9)}`, name,
        endPoint:ep,
        controlPoints:[{x:mid.x+m2ux*dDep,y:mid.y+m2uy*dDep},{x:toPos.x-arrUx*dArr,y:toPos.y-arrUy*dArr}],
        color, locked:false, waitBeforeMs:0,waitAfterMs:0,waitBeforeName:"",waitAfterName:"" },
    ];
  }
  return [makeStraight(ep, name, color)];
}

// ─── Normal curved segment ────────────────────────────────────────────────────

function buildCurvedSegment(
  fromPos: {x:number;y:number}, fromUx: number, fromUy: number,
  ball: BallPosition, io: number, rw: number, rh: number, fs: number,
  color: string, obstacles: ObstaclePolygon[],
): { lines: Line[]; rc: {x:number;y:number}; arrUx: number; arrUy: number } {
  const dx = ball.x-fromPos.x, dy = ball.y-fromPos.y, d = vecLen(dx, dy);
  const approachUx = dx/d, approachUy = dy/d;

  const offset = Math.max(io - 1, rh/2 - 1);
  const rc = clampToField(
    {x: ball.x - approachUx*offset, y: ball.y - approachUy*offset},
    rw, rh, fs,
  );

  const arrDx = ball.x-rc.x, arrDy = ball.y-rc.y, arrD = vecLen(arrDx, arrDy);
  const arrUx = arrDx/arrD, arrUy = arrDy/arrD;

  const lines = buildSafeSegment(fromPos, fromUx, fromUy, rc, arrUx, arrUy,
    tangentialPoint(rc), `Intake ${ball.id}`, color, rw, rh, fs, obstacles, ball);
  return { lines, rc, arrUx, arrUy };
}

// ─── Wall: flush strafe ───────────────────────────────────────────────────────

function buildFlushStrafe(
  fromPos: {x:number;y:number}, fromUx: number, fromUy: number,
  wallBalls: BallPosition[], wall: Wall, sweepSign: number, io: number,
  rw: number, rh: number, fs: number, color: string, obstacles: ObstaclePolygon[],
  ballRadius: number,
  viaPos?: {x:number;y:number},
): { lines: Line[]; sequence: SequenceItem[]; exitPos: {x:number;y:number}; exitUx: number; exitUy: number } {
  const lines: Line[] = [], seq: SequenceItem[] = [];
  const rcs = wallBalls.map(b => getWallRC(b, wall, io, rw, rh, fs));
  const firstRC = rcs[0], lastRC = rcs[rcs.length-1];
  const facingDeg = wallFacingDeg(wall);
  const parallelAxis = wallParallelAxis(wall);
  const axis = parallelAxis;

  // ── Compute arrival heading at firstRC ────────────────────────────────────
  // The arrival heading is the direction from fromPos to firstRC, blended
  // toward facingDeg. This ensures the robot arrives committed to the wall
  // direction rather than whatever angle it happened to approach from.
  // We also pre-angle by up to 35° in the sweep direction based on ball spread,
  // so the intake is already opening toward the next balls on entry.
  const approachDx = firstRC.x - fromPos.x, approachDy = firstRC.y - fromPos.y;
  const approachD = vecLen(approachDx, approachDy);
  const approachAngleDeg = Math.atan2(approachDy, approachDx) * 180/Math.PI;

  // Blend approach angle toward facingDeg (70% facing, 30% approach direction)
  // This avoids wild arrival angles while still being natural
  let blendedDeg = facingDeg;
  {
    let diff = approachAngleDeg - facingDeg;
    while (diff >  180) diff -= 360;
    while (diff < -180) diff += 360;
    blendedDeg = facingDeg + clamp(diff * 0.3, -45, 45);
  }

  // Pre-angle in sweep direction based on ball spread along wall
  const ballPositions = wallBalls.map(b => parallelAxis === "y" ? b.y : b.x);
  const ballSpread = Math.max(...ballPositions) - Math.min(...ballPositions);
  const maxPreAngleDeg = 35;
  const preAngleFraction = clamp(ballSpread / (2 * io), 0, 1);
  const preAngleDeg = sweepSign * preAngleFraction * maxPreAngleDeg;

  const arrivalDeg = blendedDeg + preAngleDeg;
  const arrivalRad = arrivalDeg * Math.PI / 180;
  const arrUx = Math.cos(arrivalRad), arrUy = Math.sin(arrivalRad);

  // ── Approach segment to firstRC ───────────────────────────────────────────
  if (dist(fromPos, firstRC) > 2) {
    const prevDeg = Math.atan2(fromUy, fromUx) * (180/Math.PI);

    if (viaPos) {
      // Straight line through the outlier RC directly to firstRC.
      // The robot drives in a straight shot, intake picks up the outlier ball
      // on the way, and arrives at firstRC ready to strafe.
      // One segment: fromPos → firstRC, with viaPos as a control point midway
      // only if the path deviates meaningfully from straight. Otherwise pure straight.
      const totalDx = firstRC.x - fromPos.x, totalDy = firstRC.y - fromPos.y;
      const totalD = vecLen(totalDx, totalDy);
      const totalUx = totalDx/totalD, totalUy = totalDy/totalD;

      // Check how far viaPos is from the straight fromPos→firstRC line
      const toVia = { x: viaPos.x - fromPos.x, y: viaPos.y - fromPos.y };
      const along = toVia.x * totalUx + toVia.y * totalUy;
      const perpX = toVia.x - along * totalUx, perpY = toVia.y - along * totalUy;
      const lateralOffset = Math.sqrt(perpX*perpX + perpY*perpY);

      if (lateralOffset < 4) {
        // Nearly collinear — single straight segment
        const safe = buildSafeStraightSegment(
          fromPos, firstRC, prevDeg, arrivalDeg,
          "Wall Approach", color, rw, rh, fs, obstacles);
        lines.push(...safe);
        seq.push(...safe.map(l => ({ kind:"path" as const, lineId:l.id! })));
      } else {
        // Slight lateral offset — one Bézier with minimal tension to curve through viaPos
        const t = clamp(totalD * 0.2, 4, 18);
        const cp1 = { x: fromPos.x + totalUx*t,  y: fromPos.y + totalUy*t  };
        const cp2 = { x: firstRC.x - arrUx*t,    y: firstRC.y - arrUy*t    };
        if (!pathHitsObstruction(fromPos, cp1, cp2, firstRC, rw, rh, fs, obstacles)) {
          const seg: Line = {
            id: `approach-${Math.random().toString(36).slice(2,9)}`,
            name: "Wall Approach",
            endPoint: linearPoint(firstRC, prevDeg, arrivalDeg),
            controlPoints: [cp1, cp2],
            color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"",
          };
          lines.push(seg);
          seq.push({ kind:"path" as const, lineId:seg.id! });
        } else {
          // Split into two safe straights through viaPos to avoid wall clipping
          const viaDeg = angleDeg(fromPos, viaPos);
          const seg1 = buildSafeStraightSegment(
            fromPos, viaPos, prevDeg, viaDeg,
            "Wall Approach (via)", color, rw, rh, fs, obstacles);
          const seg2 = buildSafeStraightSegment(
            viaPos, firstRC, viaDeg, arrivalDeg,
            "Wall Approach", color, rw, rh, fs, obstacles);
          lines.push(...seg1, ...seg2);
          seq.push(...seg1.map(l => ({ kind:"path" as const, lineId:l.id! })));
          seq.push(...seg2.map(l => ({ kind:"path" as const, lineId:l.id! })));
        }
      }
    } else {
      // Check if we need a curve or can go straight
      let facingDiff = arrivalDeg - approachAngleDeg;
      while (facingDiff >  180) facingDiff -= 360;
      while (facingDiff < -180) facingDiff += 360;
      const needsCurve = Math.abs(facingDiff) > 10;
      const t = clamp(approachD * 0.18, 4, 20);
      const cp1 = {x:fromPos.x+fromUx*t,   y:fromPos.y+fromUy*t  };
      const cp2 = {x:firstRC.x-arrUx*t,    y:firstRC.y-arrUy*t   };
      if (needsCurve) {
        if (pathHitsObstruction(fromPos, cp1, cp2, firstRC, rw, rh, fs, obstacles)) {
          const safeLines = buildSafeSegment(fromPos, fromUx, fromUy, firstRC, arrUx, arrUy,
            linearPoint(firstRC, prevDeg, arrivalDeg), "Wall Approach", color, rw, rh, fs, obstacles);
          lines.push(...safeLines);
          seq.push(...safeLines.map(l => ({ kind:"path" as const, lineId:l.id! })));
        } else {
          const approachLine: Line = {
            id: `approach-${Math.random().toString(36).slice(2,9)}`,
            name: "Wall Approach",
            endPoint: linearPoint(firstRC, prevDeg, arrivalDeg),
            controlPoints: [cp1, cp2],
            color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"",
          };
          lines.push(approachLine);
          seq.push({ kind:"path" as const, lineId:approachLine.id! });
        }
      } else {
        const safe = buildSafeStraightSegment(
          fromPos, firstRC, prevDeg, arrivalDeg,
          "Wall Approach", color, rw, rh, fs, obstacles);
        lines.push(...safe);
        seq.push(...safe.map(l => ({ kind:"path" as const, lineId:l.id! })));
      }
    }
  }

  // ── Strafe segment: firstRC → lastRC ──────────────────────────────────────
  // End heading: angle toward the last ball from firstRC, opening the intake.
  const lastBall = wallBalls[wallBalls.length - 1];
  const stopDist = Math.max(4, ballRadius + 2);
  const stopRC = getWallStrafeStopRC(lastBall, wall, io, rw, rh, fs, sweepSign, stopDist);

  let edgeDist = 0;
  if (wall === "left" || wall === "right") {
    edgeDist = sweepSign < 0 ? lastBall.y : (fs - lastBall.y);
  } else {
    edgeDist = sweepSign < 0 ? lastBall.x : (fs - lastBall.x);
  }
  const mag = clamp(30 + (23 - clamp(edgeDist, 0, 23)) * (55 / 23), 30, 85);
  const signedDelta = sweepSign < 0 ? -mag : mag;
  const strafeFinalDeg = facingDeg + signedDelta;

  if (dist(firstRC, lastRC) > 1) {
    const safe = buildSafeStraightSegment(
      firstRC, stopRC, arrivalDeg, strafeFinalDeg,
      "Wall Strafe", color, rw, rh, fs, obstacles);
    lines.push(...safe);
    seq.push(...safe.map(l => ({ kind:"path", lineId:l.id! })));
  }

  return { lines, sequence:seq, exitPos:stopRC,
    exitUx: axis==="x" ? sweepSign : 0, exitUy: axis==="y" ? sweepSign : 0 };
}

// ─── Ordering score ───────────────────────────────────────────────────────────

function scoreOrdering(
  startPos: {x:number;y:number}, startUx: number, startUy: number,
  order: BallPosition[], io: number, rw: number, rh: number, fs: number,
): number {
  let score=0, prev=startPos, prevUx=startUx, prevUy=startUy;
  for (const ball of order) {
    const dx=ball.x-prev.x, dy=ball.y-prev.y, d=vecLen(dx,dy);
    const ux=dx/d, uy=dy/d;
    score += d;
    const dot = prevUx*ux+prevUy*uy;
    if (dot < 0) score += (-dot)*60;
    prevUx=ux; prevUy=uy;
    prev = getRobotCenter(prev, ball, io, rw, rh, fs);
  }
  return score;
}

// ─── Main export ──────────────────────────────────────────────────────────────

export function generateOptimalBallPath(
  startPose: {x:number;y:number;headingDeg:number},
  balls: BallPosition[],
  optSettings: OptimizerSettings,
  weights: OptimizerWeights,
  maxVelocity = 40,
): OptimalPathResult {
  const startPoint = poseToPoint(startPose);

  const { robotWidth:rw, robotHeight:rh, fieldSize:fs, ballRadius, wallMargin } = optSettings;
  const obstacles: ObstaclePolygon[] = optSettings.obstacles ?? [];
  const io = optSettings.intakeOffset > 0 ? optSettings.intakeOffset : rh/2 + ballRadius;

  const reachable = filterReachableBalls(balls, io, rw, rh, fs, obstacles);
  if (reachable.length === 0) return { startPoint, lines:[], sequence:[] };

  const startPos = { x:startPose.x, y:startPose.y };
  const color = getRandomColor();

  const startRad = startPose.headingDeg * Math.PI / 180;
  let exitUx = Math.cos(startRad), exitUy = Math.sin(startRad);
  if (Math.abs(exitUx)<0.01 && Math.abs(exitUy)<0.01) {
    const dx=reachable[0].x-startPos.x, dy=reachable[0].y-startPos.y, d=vecLen(dx,dy);
    exitUx=dx/d; exitUy=dy/d;
  }

  // ── TIGHT CLUSTER: balls touching/within 1" → single straight pass ──────
  if (isTightCluster(reachable, ballRadius, rw)) {
    const cen = centroid(reachable);
    const wall = nearestWall({ id:"cen", ...cen } as BallPosition, wallMargin, fs);

    if (wall) {
      // Near a wall: single approach to centroid RC — no strafe needed.
      const cenRC = getWallRC({ id:"cen", ...cen } as BallPosition, wall, io, rw, rh, fs);
      const prevDeg = Math.atan2(exitUy, exitUx) * (180 / Math.PI);
      const facingDeg = wallFacingDeg(wall);
      const safe = buildSafeStraightSegment(
        startPos, cenRC, prevDeg, facingDeg,
        "Wall Approach", color, rw, rh, fs, obstacles);
      return { startPoint, lines:safe, sequence:safe.map(l => ({kind:"path",lineId:l.id!})) };
    }

    // Open field: drive straight through at current heading — no turning.
    // The intake width covers both balls as the robot passes through the centroid.
    {
      const dx0 = cen.x - startPos.x, dy0 = cen.y - startPos.y;
      const d0 = vecLen(dx0, dy0);
      const sweepUx = dx0/d0, sweepUy = dy0/d0;

      const rc = clampToField(
        { x: cen.x + sweepUx * (rh / 2), y: cen.y + sweepUy * (rh / 2) },
        rw, rh, fs,
      );
      const seg: Line = makeStraight(tangentialPoint(rc), "Cluster Intake", color);
      return { startPoint, lines:[seg], sequence:[{kind:"path",lineId:seg.id!}] };
    }
  }

  // ── MEDIUM CLUSTER: balls decently close → single cubic spline or wall sweep ─
  if (isMediumCluster(reachable, rw, ballRadius)) {
    const cen = centroid(reachable);
    const wall = nearestWall({ id:"cen", ...cen } as BallPosition, wallMargin, fs);

    // Best ordering by distance
    const allOrders = permutations(reachable);
    let bestOrder = allOrders[0], bestScore = Infinity;
    for (const order of allOrders) {
      const s = scoreOrdering(startPos, exitUx, exitUy, order, io, rw, rh, fs);
      if (s < bestScore) { bestScore=s; bestOrder=order; }
    }

    if (wall) {
      // Near a wall: single approach segment to the centroid RC.
      // No strafe needed — the approach angle covers all balls in the cluster.
      const cen3 = centroid(bestOrder);
      const cenRC = getWallRC({ id:"cen", ...cen3 } as BallPosition, wall, io, rw, rh, fs);
      const prevDeg = Math.atan2(exitUy, exitUx) * (180 / Math.PI);
      const facingDeg = wallFacingDeg(wall);
      const safe = buildSafeStraightSegment(
        startPos, cenRC, prevDeg, facingDeg,
        "Wall Approach", color, rw, rh, fs, obstacles);
      return { startPoint, lines:safe, sequence:safe.map(l => ({kind:"path",lineId:l.id!})) };
    }

    // Open field: just drive straight through the centroid at current heading.
    // No turning — intake width covers all balls in the cluster.
    {
      const cen2 = centroid(reachable);
      const dx = cen2.x - startPos.x, dy = cen2.y - startPos.y;
      const d = vecLen(dx, dy);
      const sweepUx = dx/d, sweepUy = dy/d;
      const rc = clampToField(
        { x: cen2.x + sweepUx * (rh/2), y: cen2.y + sweepUy * (rh/2) },
        rw, rh, fs,
      );
      const seg: Line = makeStraight(tangentialPoint(rc), "Cluster Intake", color);
      return { startPoint, lines:[seg], sequence:[{kind:"path",lineId:seg.id!}] };
    }
  }

  // ── STRAIGHT LINE INTAKE: all balls fall within intake width from a line ─
  {
    const straight = canStraightLineIntake(startPos, reachable, rw, ballRadius);
    if (straight) {
      const endRC = getRobotCenter(startPos, straight.farthest, io, rw, rh, fs);
      const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
      const endDeg = Math.atan2(straight.dirUy, straight.dirUx) * 180 / Math.PI;
      const chord = vecLen(endRC.x - startPos.x, endRC.y - startPos.y);
      const t = clamp(chord * 0.38, 6, 55);
      const cp1 = { x: startPos.x + straight.dirUx * t, y: startPos.y + straight.dirUy * t };
      const cp2 = { x: endRC.x - straight.dirUx * t,   y: endRC.y - straight.dirUy * t   };
      if (!pathHitsObstruction(startPos, cp1, cp2, endRC, rw, rh, fs, obstacles)) {
        const intakeHalfWidth = (rw / 2) - 1;
        const nearLimit = straight.maxPerp > (intakeHalfWidth + ballRadius - 0.25);
        if (nearLimit) {
          const rightBall = [...reachable].sort((a, b) => b.x - a.x)[0];
          const rightRC = getRobotCenter(startPos, rightBall, io, rw, rh, fs);
          const toRightDx = rightRC.x - startPos.x, toRightDy = rightRC.y - startPos.y;
          const toRightD = vecLen(toRightDx, toRightDy);
          const rightUx = toRightDx / toRightD, rightUy = toRightDy / toRightD;
          const rightDeg = Math.atan2(rightUy, rightUx) * 180 / Math.PI;

          const seg1 = buildSafeBezierSegment(
            startPos, straight.dirUx, straight.dirUy,
            rightRC, rightUx, rightUy,
            startDeg, rightDeg,
            "Straight Intake (via right)", color, rw, rh, fs, obstacles,
          );
          const seg2 = buildSafeBezierSegment(
            rightRC, rightUx, rightUy,
            endRC, straight.dirUx, straight.dirUy,
            rightDeg, endDeg,
            "Straight Intake", color, rw, rh, fs, obstacles,
          );
          return { startPoint, lines: [...seg1, ...seg2], sequence: [...seg1, ...seg2].map(l => ({ kind:"path", lineId: l.id! })) };
        }
        const seg: Line = makeStraight(tangentialPoint(endRC), "Straight Intake", color);
        return { startPoint, lines: [seg], sequence: [{ kind:"path", lineId: seg.id! }] };
      }
    }
  }

  // ── CASE 2: All 3 far from any wall → single spline from below/top ───────
  if (reachable.length === 3) {
    const triangle = detectTriangleStrafePattern(reachable, rw);
    if (triangle) {
      // Let triangle handler build bezier intake (2->1) then strafe to 3.
      // Skip spline in this layout.
    } else {
    const nearAnyWall = reachable.some(b => nearestWall(b, wallMargin, fs));
    if (!nearAnyWall) {
      const sortedByY = [...reachable].sort((a, b) => a.y - b.y);
      const bottom = sortedByY[0];
      const top = sortedByY[2];
      const clearance = rh + 4;
      const canApproachFromBelow = bottom.y > clearance;
      const canApproachFromAbove = fs - top.y > clearance;

      if (canApproachFromBelow || canApproachFromAbove) {
        const fromBelow = canApproachFromBelow;
        const order = fromBelow ? sortedByY : [...sortedByY].reverse();
        const offset = io - 1;
        const rcs = order.map((b) => {
          const rc = fromBelow
            ? clampToField({ x: b.x, y: b.y - offset }, rw, rh, fs)
            : clampToField({ x: b.x, y: b.y + offset }, rw, rh, fs);
          if (obstacles.length > 0 && robotHitsObstacle(rc.x, rc.y, rw/2, rh/2, obstacles)) {
            return clampToField({ x: rc.x - rw, y: rc.y }, rw, rh, fs);
          }
          return rc;
        });

        const splineLines = buildBezierChain(startPos, rcs, "Ball Spline", color, rw, rh, fs, obstacles, true);
        return { startPoint, lines: splineLines, sequence: splineLines.map(l => ({ kind:"path", lineId: l.id! })) };
      }
    }
    }
  }

  // Detect dominant wall
  const ballWalls = reachable.map(b => nearestWall(b, wallMargin, fs));
  const wallCount: Partial<Record<Wall,number>> = {};
  ballWalls.forEach(w => { if (w) wallCount[w]=(wallCount[w]??0)+1; });
  const topEntry = (Object.entries(wallCount) as [Wall,number][]).sort((a,b)=>b[1]-a[1])[0];
  const dominantWall: Wall|null = topEntry && topEntry[1]>=2 ? topEntry[0] : null;

  // ── Case-specific right-wall logic (FTC artifacts) ───────────────────────
  if (dominantWall === "right" && reachable.length === 3) {
    const sorted = [...reachable].sort((a, b) => b.y - a.y);
    const [b1, b2, b3] = sorted; // b1=top, b2=middle, b3=bottom
    const nearTol = wallMargin + 2;
    const nearWall = (b: BallPosition) => (fs - b.x) <= nearTol;
    const b1wall = nearWall(b1), b2wall = nearWall(b2), b3wall = nearWall(b3);

    const wallRC = (b: BallPosition) => getWallRC(b, "right", io, rw, rh, fs);

    // Case 1: all 3 near wall — start at top, open intake quickly while strafing down
    if (b1wall && b2wall && b3wall) {
      const inset = 2;
      const rc1 = applyWallInset(wallRC(b1), "right", inset, rw, rh, fs);
      const stopDist = Math.max(4, ballRadius + 2);
      const stopRC = applyWallInset(
        getWallStrafeStopRC(b3, "right", io, rw, rh, fs, -1, stopDist),
        "right", inset, rw, rh, fs,
      );
      const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
      const approach = buildSafeBezierSegment(
        startPos, exitUx, exitUy,
        rc1, 1, 0,
        startDeg, 0,
        "Wall Approach", color, rw, rh, fs, obstacles);

      const edgeDist = clamp(b3.y, 0, 23);
      const mag = clamp(30 + (23 - edgeDist) * (55 / 23), 30, 85);
      const strafeFinalDeg = -mag;
      const strafe = buildSafeStraightSegment(
        rc1, stopRC, 0, strafeFinalDeg,
        "Wall Strafe", color, rw, rh, fs, obstacles);

      const lines = [...approach, ...strafe];
      const sequence = lines.map(l => ({ kind:"path" as const, lineId:l.id! }));
      return { startPoint, lines, sequence };
    }

    // Case 3: b1 near wall, b2+b3 far — approach b3 angled toward b1, sweep b2, hook b1
    if (b1wall && !b2wall && !b3wall) {
      const toB1Deg = angleDeg(b3, b1);
      const fromBelowDeg = 90;
      const b3ArrivalDeg = toB1Deg * 0.6 + fromBelowDeg * 0.4;
      const b3ArrRad = b3ArrivalDeg * Math.PI / 180;
      const b3ArrUx = Math.cos(b3ArrRad), b3ArrUy = Math.sin(b3ArrRad);

      const rc3 = clampToField(
        { x: b3.x - b3ArrUx*(io-1), y: b3.y - b3ArrUy*(io-1) }, rw, rh, fs);

      const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
      const approach = buildSafeBezierSegment(
        startPos, exitUx, exitUy,
        rc3, b3ArrUx, b3ArrUy,
        startDeg, b3ArrivalDeg,
        "Approach b3", color, rw, rh, fs, obstacles);

      const toWallDeg = 0;
      const b2HeadingDeg = b3ArrivalDeg * 0.4 + toWallDeg * 0.6;
      const b2HeadRad = b2HeadingDeg * Math.PI / 180;
      const b2HeadUx = Math.cos(b2HeadRad), b2HeadUy = Math.sin(b2HeadRad);
      const rc2 = clampToField(
        { x: b2.x - b2HeadUx*(io-1), y: b2.y - b2HeadUy*(io-1) }, rw, rh, fs);

      const sweep = buildSafeBezierSegment(
        rc3, b3ArrUx, b3ArrUy,
        rc2, b2HeadUx, b2HeadUy,
        b3ArrivalDeg, b2HeadingDeg,
        "Sweep b2", color, rw, rh, fs, obstacles);

      const rc1 = wallRC(b1);
      const hookChord = vecLen(rc1.x - rc2.x, rc1.y - rc2.y);
      const hook = buildSafeBezierSegment(
        rc2, b2HeadUx, b2HeadUy,
        rc1, 1, 0,
        b2HeadingDeg, 0,
        "Hook b1", color, rw, rh, fs, obstacles,
        clamp(hookChord * 0.25, 4, 20),
        clamp(hookChord * 0.7, 8, 40),
      );

      const lines = [...approach, ...sweep, ...hook];
      const sequence = lines.map(l => ({ kind:"path" as const, lineId:l.id! }));
      return { startPoint, lines, sequence };
    }

    // Case 2.5: b3 near wall, b1+b2 far — mirror of case 3
    if (!b1wall && !b2wall && b3wall) {
      const toB3Deg = angleDeg(b1, b3);
      const fromBelowDeg = 90;
      const b1ArrivalDeg = toB3Deg * 0.6 + fromBelowDeg * 0.4;
      const b1ArrRad = b1ArrivalDeg * Math.PI / 180;
      const b1ArrUx = Math.cos(b1ArrRad), b1ArrUy = Math.sin(b1ArrRad);

      const rc1 = clampToField(
        { x: b1.x - b1ArrUx*(io-1), y: b1.y - b1ArrUy*(io-1) }, rw, rh, fs);

      const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
      const approach = buildSafeBezierSegment(
        startPos, exitUx, exitUy,
        rc1, b1ArrUx, b1ArrUy,
        startDeg, b1ArrivalDeg,
        "Approach b1", color, rw, rh, fs, obstacles);

      const toWallDeg = 0;
      const b2HeadingDeg = b1ArrivalDeg * 0.4 + toWallDeg * 0.6;
      const b2HeadRad = b2HeadingDeg * Math.PI / 180;
      const b2HeadUx = Math.cos(b2HeadRad), b2HeadUy = Math.sin(b2HeadRad);
      const rc2 = clampToField(
        { x: b2.x - b2HeadUx*(io-1), y: b2.y - b2HeadUy*(io-1) }, rw, rh, fs);

      const sweep = buildSafeBezierSegment(
        rc1, b1ArrUx, b1ArrUy,
        rc2, b2HeadUx, b2HeadUy,
        b1ArrivalDeg, b2HeadingDeg,
        "Sweep b2", color, rw, rh, fs, obstacles);

      const rc3 = wallRC(b3);
      const hookChord = vecLen(rc3.x - rc2.x, rc3.y - rc2.y);
      const hook = buildSafeBezierSegment(
        rc2, b2HeadUx, b2HeadUy,
        rc3, 1, 0,
        b2HeadingDeg, 0,
        "Hook b3", color, rw, rh, fs, obstacles,
        clamp(hookChord * 0.25, 4, 20),
        clamp(hookChord * 0.7, 8, 40),
      );

      const lines = [...approach, ...sweep, ...hook];
      const sequence = lines.map(l => ({ kind:"path" as const, lineId:l.id! }));
      return { startPoint, lines, sequence };
    }

    // Case 4: b1 near wall, b2 far, b3 near — curve to b2→b1, then strafe down
    if (b1wall && !b2wall && b3wall) {
      const toB1Deg = angleDeg(b2, b1);
      const fromBelowDeg = 90;
      const b2ArrivalDeg = toB1Deg * 0.6 + fromBelowDeg * 0.4;
      const b2ArrRad = b2ArrivalDeg * Math.PI / 180;
      const b2ArrUx = Math.cos(b2ArrRad), b2ArrUy = Math.sin(b2ArrRad);

      const rc2 = clampToField(
        { x: b2.x - b2ArrUx*(io-1), y: b2.y - b2ArrUy*(io-1) }, rw, rh, fs);
      const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
      const approach = buildSafeBezierSegment(
        startPos, exitUx, exitUy,
        rc2, b2ArrUx, b2ArrUy,
        startDeg, b2ArrivalDeg,
        "Approach b2", color, rw, rh, fs, obstacles);

      const rc1 = wallRC(b1);
      const hookChord = vecLen(rc1.x - rc2.x, rc1.y - rc2.y);
      const hook = buildSafeBezierSegment(
        rc2, b2ArrUx, b2ArrUy,
        rc1, 1, 0,
        b2ArrivalDeg, 0,
        "Hook b1", color, rw, rh, fs, obstacles,
        clamp(hookChord * 0.25, 4, 20),
        clamp(hookChord * 0.7, 8, 40),
      );

      const stopDist = Math.max(4, ballRadius + 2);
      const stopRC = getWallStrafeStopRC(b3, "right", io, rw, rh, fs, -1, stopDist);
      const edgeDist = clamp(b3.y, 0, 23);
      const mag = clamp(30 + (23 - edgeDist) * (55 / 23), 30, 85);
      const strafeFinalDeg = -mag;
      const strafe = buildSafeStraightSegment(
        rc1, stopRC, 0, strafeFinalDeg,
        "Wall Strafe", color, rw, rh, fs, obstacles);

      const lines = [...approach, ...hook, ...strafe];
      const sequence = lines.map(l => ({ kind:"path" as const, lineId:l.id! }));
      return { startPoint, lines, sequence };
    }
  }

  // ── WALL SWEEP ────────────────────────────────────────────────────────────
  // Smart wall logic: only truly wall-hugging balls (within wallMargin) get
  // the strafe treatment. Balls further from the wall get a normal curved
  // approach even if they're on the "wall side" of the field.
  if (dominantWall) {
    let wallBalls    = reachable.filter(b => nearestWall(b, wallMargin, fs) === dominantWall);
    let outlierBalls = reachable.filter(b => nearestWall(b, wallMargin, fs) !== dominantWall);

    // ── Promote lateral middle ball to outlier (viaPos) ───────────────────
    // If all 3 balls are within wallMargin but the middle one (by parallel axis)
    // is significantly further from the wall than the two endpoints, it should be
    // intaked as a via-point on the way between the endpoints rather than treated
    // as a pure strafe waypoint. This is the "triangle" case: top-right, middle-left,
    // bottom-right — the robot should sweep top→middle→bottom with the middle ball
    // approached via a natural curve rather than a straight strafe line.
    if (wallBalls.length === 3 && outlierBalls.length === 0) {
      const parallelAxis = wallParallelAxis(dominantWall);
      const sorted3 = [...wallBalls].sort((a, b) =>
        parallelAxis === "y" ? a.y - b.y : a.x - b.x);
      const midBall = sorted3[1];
      const endBall0 = sorted3[0], endBall2 = sorted3[2];
      const midDist  = ballDistFromWall(midBall,  dominantWall, fs);
      const end0Dist = ballDistFromWall(endBall0, dominantWall, fs);
      const end2Dist = ballDistFromWall(endBall2, dominantWall, fs);
      const endAvgDist = (end0Dist + end2Dist) / 2;
      // If middle ball is more than 3" further from wall than the endpoint average,
      // promote it to outlier so it becomes a viaPos in the strafe
      if (midDist > endAvgDist + 3) {
        outlierBalls = [midBall];
        wallBalls    = [endBall0, endBall2];
      }
    }

    // Special: right-right-left (top+mid near right wall, bottom left)
    if (dominantWall === "right" && wallBalls.length === 2 && outlierBalls.length === 1) {
      const rightSorted = [...wallBalls].sort((a, b) => b.y - a.y);
      const bottomOutlier = outlierBalls[0];
      if (bottomOutlier.y <= rightSorted[1].y) {
        const allLines: Line[] = [], allSeq: SequenceItem[] = [];
        let cur = startPos, curUx = exitUx, curUy = exitUy;
        const intake = buildCurvedSegment(
          cur, curUx, curUy, bottomOutlier, io, rw, rh, fs, color, obstacles);
        allLines.push(...intake.lines);
        allSeq.push(...intake.lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
        cur = intake.rc; curUx = intake.arrUx; curUy = intake.arrUy;

        const { sorted: sweepBalls, sweepSign } = sortForSweep(rightSorted, dominantWall, cur, fs);
        const result = buildFlushStrafe(
          cur, curUx, curUy, sweepBalls, dominantWall, sweepSign,
          io, rw, rh, fs, color, obstacles, ballRadius);
        allLines.push(...result.lines); allSeq.push(...result.sequence);
        return { startPoint, lines: allLines, sequence: allSeq };
      }
    }

    // Sort wall balls for the sweep
    const { sorted: sweepBalls, sweepSign } = sortForSweep(wallBalls, dominantWall, startPos, fs);
    const firstWallRC = getWallRC(sweepBalls[0], dominantWall, io, rw, rh, fs);

    const allLines: Line[] = [], allSeq: SequenceItem[] = [];
    let cur = startPos, curUx = exitUx, curUy = exitUy;

    // Handle outlier balls first with normal curved segments
    if (outlierBalls.length > 0) {
      // Sort outliers by total travel cost (nearest first, then toward wall)
      const sortedOutliers = [...outlierBalls].sort((a, b) => {
        const rcA = getRobotCenter(cur, a, io, rw, rh, fs);
        const rcB = getRobotCenter(cur, b, io, rw, rh, fs);
        return (dist(cur,rcA)+dist(rcA,firstWallRC)) - (dist(cur,rcB)+dist(rcB,firstWallRC));
      });

      if (sortedOutliers.length === 1) {
        // Single outlier: always pass its RC as viaPos so the path curves
        // smoothly through the outlier ball ON THE WAY to the wall strafe.
        // This handles both "b is lateral to sweep" and "b is before wall" cases.
        const ob = sortedOutliers[0];
        const obRC = getRobotCenter(cur, ob, io, rw, rh, fs);
        const result = buildFlushStrafe(
          cur, curUx, curUy, sweepBalls, dominantWall, sweepSign,
          io, rw, rh, fs, color, obstacles, ballRadius, obRC);
        allLines.push(...result.lines); allSeq.push(...result.sequence);
      } else {
        // Multiple outliers: normal curved segment to each, then wall strafe
        for (const ob of sortedOutliers) {
          const { lines:obLines, rc, arrUx, arrUy } = buildCurvedSegment(
            cur, curUx, curUy, ob, io, rw, rh, fs, color, obstacles);
          allLines.push(...obLines);
          allSeq.push(...obLines.map(l => ({ kind:"path" as const, lineId:l.id! })));
          const toW = vecLen(firstWallRC.x-rc.x, firstWallRC.y-rc.y);
          curUx=(firstWallRC.x-rc.x)/toW; curUy=(firstWallRC.y-rc.y)/toW;
          cur=rc;
        }
        const result = buildFlushStrafe(
          cur, curUx, curUy, sweepBalls, dominantWall, sweepSign,
          io, rw, rh, fs, color, obstacles, ballRadius);
        allLines.push(...result.lines); allSeq.push(...result.sequence);
      }
    } else {
      const result = buildFlushStrafe(
        cur, curUx, curUy, sweepBalls, dominantWall, sweepSign,
        io, rw, rh, fs, color, obstacles, ballRadius);
      allLines.push(...result.lines); allSeq.push(...result.sequence);
    }

    return { startPoint, lines:allLines, sequence:allSeq };
  }

  // ── TRIANGLE STRAFE PATTERN (FTC artifacts) ─────────────────────────────
  // If balls form a left+top+bottom triangle, take left → top, then strafe to bottom.
  const triangle = detectTriangleStrafePattern(reachable, rw);
  if (triangle) {
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    let cur = startPos;
    let curUx = exitUx, curUy = exitUy;

    // Triangle orientation where top+bottom are aligned and left is the middle:
    // intake left (ball-2), then top (ball-1), then strafe to bottom (ball-3).
    const rightColumnAligned = Math.abs(triangle.top.x - triangle.bottom.x) <= Math.max(6, rw * 0.35);
    const leftIsMiddle = triangle.left.y < triangle.top.y && triangle.left.y > triangle.bottom.y;
    if (rightColumnAligned && leftIsMiddle) {
      const leftRes = buildCurvedSegment(
        cur, curUx, curUy, triangle.left, io, rw, rh, fs, color, obstacles);
      lines.push(...leftRes.lines);
      sequence.push(...leftRes.lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
      cur = leftRes.rc; curUx = leftRes.arrUx; curUy = leftRes.arrUy;

      const topRes = buildCurvedSegment(
        cur, curUx, curUy, triangle.top, io, rw, rh, fs, color, obstacles);
      lines.push(...topRes.lines);
      sequence.push(...topRes.lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
      cur = topRes.rc; curUx = topRes.arrUx; curUy = topRes.arrUy;

      const bottomRC = getRobotCenter(cur, triangle.bottom, io, rw, rh, fs);
      const edgeDist = clamp(triangle.bottom.y, 0, 23);
      const mag = clamp(30 + (23 - edgeDist) * (55 / 23), 30, 85);
      const endDeg = -mag;
      const startDeg = Math.atan2(curUy, curUx) * 180 / Math.PI;
      const strafe = buildSafeBezierSegment(
        cur, 0, -1,
        bottomRC, 0, -1,
        startDeg, endDeg,
        "Triangle Strafe", color, rw, rh, fs, obstacles,
        clamp(dist(cur, bottomRC) * 0.25, 6, 26),
        clamp(dist(cur, bottomRC) * 0.45, 8, 36),
      );
      lines.push(...strafe);
      sequence.push(...strafe.map(l => ({ kind:"path" as const, lineId:l.id! })));
      return { startPoint, lines, sequence };
    }

    const leftRes = buildCurvedSegment(
      cur, curUx, curUy, triangle.left, io, rw, rh, fs, color, obstacles);
    lines.push(...leftRes.lines);
    sequence.push(...leftRes.lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
    cur = leftRes.rc; curUx = leftRes.arrUx; curUy = leftRes.arrUy;

    const topRC = getRobotCenter(cur, triangle.top, io, rw, rh, fs);
    const bottomRC = getRobotCenter(topRC, triangle.bottom, io, rw, rh, fs);

    // If top is close to the path from cur→bottomRC, merge top+bottom into one curve
    const pathDx = bottomRC.x - cur.x, pathDy = bottomRC.y - cur.y;
    const pathLen = vecLen(pathDx, pathDy);
    const pathUx = pathDx / pathLen, pathUy = pathDy / pathLen;
    const toTopDx = topRC.x - cur.x, toTopDy = topRC.y - cur.y;
    const along = toTopDx * pathUx + toTopDy * pathUy;
    const latX = toTopDx - along * pathUx, latY = toTopDy - along * pathUy;
    const lateral = Math.sqrt(latX * latX + latY * latY);
    const topCloseToLine = lateral <= Math.max(5, rw * 0.6);

    const angDot = ((toTopDx / vecLen(toTopDx, toTopDy)) * pathUx) + ((toTopDy / vecLen(toTopDx, toTopDy)) * pathUy);
    const angDeg = Math.acos(clamp(angDot, -1, 1)) * 180 / Math.PI;

    if (topCloseToLine && angDeg < 35) {
      const toBottomDx = triangle.bottom.x - bottomRC.x;
      const toBottomDy = triangle.bottom.y - bottomRC.y;
      const toBottomD = vecLen(toBottomDx, toBottomDy);
      const arrUx = toBottomDx / toBottomD, arrUy = toBottomDy / toBottomD;
      const startDeg = Math.atan2(curUy, curUx) * 180 / Math.PI;
      const endDeg = Math.atan2(arrUy, arrUx) * 180 / Math.PI;
      const merged = buildSafeBezierSegment(
        cur, curUx, curUy,
        bottomRC, arrUx, arrUy,
        startDeg, endDeg,
        "Intake top+bottom", color, rw, rh, fs, obstacles,
      );
      lines.push(...merged);
      sequence.push(...merged.map(l => ({ kind:"path" as const, lineId:l.id! })));
      cur = bottomRC; curUx = arrUx; curUy = arrUy;
    } else {
      const topRes = buildCurvedSegment(
        cur, curUx, curUy, triangle.top, io, rw, rh, fs, color, obstacles);
      lines.push(...topRes.lines);
      sequence.push(...topRes.lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
      cur = topRes.rc; curUx = topRes.arrUx; curUy = topRes.arrUy;

      // Strafe to bottom while opening the heading (like wall strafe)
      const edgeDist = clamp(triangle.bottom.y, 0, 23);
      const mag = clamp(30 + (23 - edgeDist) * (55 / 23), 30, 85);
      const endDeg = -mag;
      const startDeg = Math.atan2(curUy, curUx) * 180 / Math.PI;
      const strafe = buildSafeStraightSegment(
        cur, bottomRC, startDeg, endDeg,
        "Triangle Strafe", color, rw, rh, fs, obstacles);
      lines.push(...strafe);
      sequence.push(...strafe.map(l => ({ kind:"path" as const, lineId:l.id! })));
      cur = bottomRC;
    }

    return { startPoint, lines, sequence };
  }

  // ── STRAIGHT VERTICAL STRAFE (line of 3 balls) ──────────────────────────
  const verticalStrafe = detectVerticalStrafePattern(reachable, rw);
  if (verticalStrafe) {
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    const topRC = getRobotCenter(startPos, verticalStrafe.top, io, rw, rh, fs);
    const bottomRC = getRobotCenter(topRC, verticalStrafe.bottom, io, rw, rh, fs);

    const nearAnyWall = reachable.some(b => nearestWall(b, wallMargin, fs));
    if (!nearAnyWall) {
      // Far from wall: come from top/bottom (whichever has more room), then straight through
      const clearanceTop = fs - verticalStrafe.top.y;
      const clearanceBottom = verticalStrafe.bottom.y;
      const fromAbove = clearanceTop >= clearanceBottom;
      const clearance = rh + 6;
      const preRC = fromAbove
        ? clampToField({ x: topRC.x, y: topRC.y + clearance }, rw, rh, fs)
        : clampToField({ x: bottomRC.x, y: bottomRC.y - clearance }, rw, rh, fs);
      const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
      const preDeg = fromAbove ? -90 : 90;
      const preApproach = buildSafeBezierSegment(
        startPos, exitUx, exitUy,
        preRC, 0, fromAbove ? -1 : 1,
        startDeg, preDeg,
        "Line Pre-Approach", color, rw, rh, fs, obstacles,
      );
      lines.push(...preApproach);
      sequence.push(...preApproach.map(l => ({ kind:"path" as const, lineId:l.id! })));

      const endRC = fromAbove ? bottomRC : topRC;
      const straight: Line = makeStraight(
        linearPoint(endRC, preDeg, preDeg),
        "Line Through", color,
      );
      lines.push(straight);
      sequence.push({ kind:"path" as const, lineId: straight.id! });
      return { startPoint, lines, sequence };
    }

    const startDeg = Math.atan2(exitUy, exitUx) * 180 / Math.PI;
    // Approach top ball with a slight open heading
    const approach = buildSafeBezierSegment(
      startPos, exitUx, exitUy,
      topRC, 1, 0,
      startDeg, -10,
      "Line Approach", color, rw, rh, fs, obstacles,
    );

    const baseEndDeg = angleDeg(topRC, bottomRC);
    // End heading opens toward bottom; default -35, but allow steeper if needed
    let endDeg = Math.min(-35, baseEndDeg);
    endDeg = clamp(endDeg, -80, -10);

    const strafeLine: Line = makeStraight(
      linearPoint(bottomRC, -10, endDeg),
      "Line Strafe",
      color,
    );

    lines.push(...approach, strafeLine);
    sequence.push(...lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
    return { startPoint, lines, sequence };
  }

  // ── TWO-LEFT / ONE-RIGHT PATTERN ─────────────────────────────────────────
  const twoLeft = detectTwoLeftOneRightPattern(reachable, rw);
  if (twoLeft) {
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    let cur = startPos;
    let curUx = exitUx, curUy = exitUy;

    const topRC = getRobotCenter(cur, twoLeft.topLeft, io, rw, rh, fs);
    const bottomRC = getRobotCenter(topRC, twoLeft.bottomLeft, io, rw, rh, fs);

    // Approach top-left while opening heading toward the right outlier
    const toRightDeg = angleDeg(twoLeft.topLeft, twoLeft.right);
    const fromAboveDeg = -90;
    const topArrivalDeg = toRightDeg * 0.6 + fromAboveDeg * 0.4;
    const topArrRad = topArrivalDeg * Math.PI / 180;
    const topArrUx = Math.cos(topArrRad), topArrUy = Math.sin(topArrRad);
    const startDeg = Math.atan2(curUy, curUx) * 180 / Math.PI;

    const clearance = rh + 6;
    const canComeFromAbove = (fs - twoLeft.topLeft.y) > clearance;
    if (canComeFromAbove) {
      const preRC = clampToField({ x: topRC.x, y: topRC.y + clearance }, rw, rh, fs);
      const preApproach = buildSafeBezierSegment(
        cur, curUx, curUy,
        preRC, 0, -1,
        startDeg, -90,
        "Left Pre-Approach", color, rw, rh, fs, obstacles);
      lines.push(...preApproach);
      sequence.push(...preApproach.map(l => ({ kind:"path" as const, lineId:l.id! })));
      cur = preRC; curUx = 0; curUy = -1;
    }

    const approach = buildSafeBezierSegment(
      cur, curUx, curUy,
      topRC, topArrUx, topArrUy,
      startDeg, topArrivalDeg,
      "Left Approach", color, rw, rh, fs, obstacles);
    lines.push(...approach);
    sequence.push(...approach.map(l => ({ kind:"path" as const, lineId:l.id! })));
    cur = topRC; curUx = topArrUx; curUy = topArrUy;

    // Strafe down the left pair with opening heading
    const edgeDist = clamp(twoLeft.bottomLeft.y, 0, 23);
    const mag = clamp(30 + (23 - edgeDist) * (55 / 23), 30, 85);
    const endDeg = -mag;
    const strafe = buildSafeBezierSegment(
      cur, 0, -1,
      bottomRC, 0, -1,
      topArrivalDeg, endDeg,
      "Left Strafe", color, rw, rh, fs, obstacles,
      clamp(dist(cur, bottomRC) * 0.25, 6, 26),
      clamp(dist(cur, bottomRC) * 0.45, 8, 36),
    );
    lines.push(...strafe);
    sequence.push(...strafe.map(l => ({ kind:"path" as const, lineId:l.id! })));

    return { startPoint, lines, sequence };
  }

  // ── TWO-RIGHT / ONE-LEFT PATTERN ────────────────────────────────────────
  const twoRight = detectTwoRightOneLeftPattern(reachable, rw);
  if (twoRight) {
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    let cur = startPos;
    let curUx = exitUx, curUy = exitUy;

    const bottomRC = getRobotCenter(cur, twoRight.bottomRight, io, rw, rh, fs);
    const topRC = getRobotCenter(bottomRC, twoRight.topRight, io, rw, rh, fs);

    // Intake bottom-right first
    const bottomRes = buildCurvedSegment(
      cur, curUx, curUy, twoRight.bottomRight, io, rw, rh, fs, color, obstacles);
    lines.push(...bottomRes.lines);
    sequence.push(...bottomRes.lines.map(l => ({ kind:"path" as const, lineId:l.id! })));
    cur = bottomRes.rc; curUx = bottomRes.arrUx; curUy = bottomRes.arrUy;

    // Strafe up the right pair with opening heading
    const edgeDist = clamp(fs - twoRight.topRight.y, 0, 23);
    const mag = clamp(30 + (23 - edgeDist) * (55 / 23), 30, 85);
    const endDeg = mag; // open toward top
    const strafe = buildSafeBezierSegment(
      cur, 0, 1,
      topRC, 0, 1,
      bottomArrivalDeg, endDeg,
      "Right Strafe", color, rw, rh, fs, obstacles,
      clamp(dist(cur, topRC) * 0.25, 6, 26),
      clamp(dist(cur, topRC) * 0.45, 8, 36),
    );
    lines.push(...strafe);
    sequence.push(...strafe.map(l => ({ kind:"path" as const, lineId:l.id! })));

    return { startPoint, lines, sequence };
  }

  // ── NORMAL ────────────────────────────────────────────────────────────────
  const allOrders = permutations(reachable);
  let bestOrder = allOrders[0], bestScore = Infinity;
  for (const order of allOrders) {
    const s = scoreOrdering(startPos, exitUx, exitUy, order, io, rw, rh, fs);
    if (s < bestScore) { bestScore=s; bestOrder=order; }
  }

  // ── COLLINEAR CHECK: if robot centers are nearly in a line, use straight ──
  const bestRCs = bestOrder.map(ball => getRobotCenter(startPos, ball, io, rw, rh, fs));
  if (areRobotCentersCollinear([startPos, ...bestRCs])) {
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    let cur = startPos, curUx = exitUx, curUy = exitUy;
    for (const ball of bestOrder) {
      const rc = getRobotCenter(cur, ball, io, rw, rh, fs);
      const seg = makeStraight(tangentialPoint(rc), `Intake ${ball.id}`, color);
      lines.push(seg);
      sequence.push({ kind:"path", lineId:seg.id! });
      cur = rc;
      const dx=ball.x-rc.x, dy=ball.y-rc.y, d=vecLen(dx,dy);
      curUx=dx/d; curUy=dy/d;
    }
    return { startPoint, lines, sequence };
  }

  // ── TRIANGLE / NORMAL: one curved segment per ball ────────────────────────
  // For each ball, compute the approach direction from the previous RC.
  // If consecutive approach angles differ by more than SPLIT_THRESHOLD degrees,
  // that ball starts a NEW path — this ensures the robot has fully committed to
  // the correct heading before reaching each ball rather than trying to curve
  // through a large angle in a single segment.
  const SPLIT_THRESHOLD_DEG = 90;

  const lines: Line[] = [], sequence: SequenceItem[] = [];
  let cur = startPos;
  let curUx = exitUx, curUy = exitUy;
  let i = 0;

  while (i < bestOrder.length) {
    const ball = bestOrder[i];

    // Compute approach direction from current position to this ball
    const dxApp = ball.x - cur.x, dyApp = ball.y - cur.y;
    const dApp = vecLen(dxApp, dyApp);
    const appUx = dxApp/dApp, appUy = dyApp/dApp;

    // Angle between current exit direction and needed approach direction
    const dot = curUx*appUx + curUy*appUy;
    const angleDeg = Math.acos(clamp(dot, -1, 1)) * 180/Math.PI;

    // If the turn needed is large AND there are remaining balls after this one,
    // check if the NEXT ball also requires a big turn from here — if so, this
    // is a triangle configuration and we should treat each ball as its own path.
    const isBigTurn = angleDeg > SPLIT_THRESHOLD_DEG;

    if (isBigTurn && i > 0) {
      // Big heading change needed — build this as its own clean curved segment.
      // The key fix: pass the BALL position as ballPos to buildSafeSegment so
      // lateral offset is computed correctly even when ball is directly ahead of rc.
      const { lines:segs, rc, arrUx, arrUy } = buildCurvedSegment(
        cur, curUx, curUy, ball, io, rw, rh, fs, color, obstacles);
      lines.push(...segs);
      sequence.push(...segs.map(l => ({ kind:"path", lineId:l.id! })));
      cur=rc; curUx=arrUx; curUy=arrUy;
      i++;
      continue;
    }

    // Check if next ball can be merged (collinear / small angle)
    if (i + 1 < bestOrder.length) {
      const next = bestOrder[i + 1];

      const dx0 = ball.x-cur.x, dy0 = ball.y-cur.y, d0 = vecLen(dx0, dy0);
      const u0x = dx0/d0, u0y = dy0/d0;
      const rc0 = clampToField({x:ball.x-u0x*io, y:ball.y-u0y*io}, rw, rh, fs);

      const dx1 = next.x-rc0.x, dy1 = next.y-rc0.y, d1 = vecLen(dx1, dy1);
      const u1x = dx1/d1, u1y = dy1/d1;
      const rc1exact = clampToField({x:next.x-u1x*io, y:next.y-u1y*io}, rw, rh, fs);

      // Check angle between approach to ball and approach to next ball from rc0
      const nextDot = u0x*u1x + u0y*u1y;
      const nextAngle = Math.acos(clamp(nextDot,-1,1))*180/Math.PI;

      // Also check detour cost
      const dCurRc1 = dist(cur, rc1exact);
      const dCurRc0 = dist(cur, rc0);
      const dRc0Rc1 = dist(rc0, rc1exact);
      const detour = (dCurRc0 + dRc0Rc1) - dCurRc1;

      if (detour < 8 && nextAngle < SPLIT_THRESHOLD_DEG) {
        const arrDx = rc1exact.x-cur.x, arrDy = rc1exact.y-cur.y, arrD = vecLen(arrDx, arrDy);
        const arrUx = arrDx/arrD, arrUy = arrDy/arrD;
        const t = clamp(arrD*0.38, 6, 55);
        const cp1 = {x:cur.x+curUx*t, y:cur.y+curUy*t};
        const cp2 = {x:rc1exact.x-arrUx*t, y:rc1exact.y-arrUy*t};

        if (!pathHitsObstruction(cur, cp1, cp2, rc1exact, rw, rh, fs, obstacles)) {
          const seg: Line = {
            id: `curve-${Math.random().toString(36).slice(2,9)}`,
            name: `Intake ${ball.id}+${next.id}`,
            endPoint: tangentialPoint(rc1exact),
            controlPoints: [cp1, cp2],
            color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"",
          };
          lines.push(seg);
          sequence.push({ kind:"path", lineId:seg.id! });
          cur=rc1exact; curUx=u1x; curUy=u1y;
          i += 2;
          continue;
        }
      }
    }

    // Default: one curved segment for this ball
    const { lines:segs, rc, arrUx, arrUy } = buildCurvedSegment(
      cur, curUx, curUy, ball, io, rw, rh, fs, color, obstacles);
    lines.push(...segs);
    sequence.push(...segs.map(l => ({ kind:"path", lineId:l.id! })));
    cur=rc; curUx=arrUx; curUy=arrUy;
    i++;
  }
  return { startPoint, lines, sequence };
}

function poseToPoint(pose: {x:number;y:number;headingDeg:number}): Point {
  return {x:pose.x, y:pose.y, heading:"tangential", reverse:false, locked:false} as Point;
}

// ─── FAR Path Generator ───────────────────────────────────────────────────────
//
// Generates an optimized path for the FAR (right-wall) scoring position on RED side.
// Fixed start: (87.5, 15.928571428571436, 0°) — near bottom, facing right (east).
//
// Balls are sorted by Y descending:
//   ball1 = top    (highest Y) — "left"   from robot perspective
//   ball2 = middle              — "middle"
//   ball3 = bottom (lowest Y)  — "right"  from robot perspective
//
// "Near wall" = x >= fieldSize - wallMargin (default: x >= 136 for 8" margin)
// "Far from wall" = x < fieldSize - wallMargin
//
// Cases:
//   Case 1:   All 3 near wall — start at ball1, strafe down to ball3, heading opens fast
//   Case 2:   All 3 far from wall — one spline from below through ball3→ball2→ball1
//   Case 3:   Ball1 near wall, ball2+ball3 far — approach ball3 angled upper-right,
//             sweep through ball2, hook into ball1
//   Case 2.5: Ball3 near wall, ball1+ball2 far — approach ball1 angled lower-right,
//             sweep through ball2, hook into ball3
//   Case 4:   Ball1+ball3 near wall, ball2 far (triangle) — curve to ball2→ball1,
//             strafe down to ball3 with opening heading

export const FAR_START_RED: { x: number; y: number; headingDeg: number } = {
  x: 87.5,
  y: 15.928571428571436,
  headingDeg: 0,
};

export interface FARPathOptions {
  wallMargin?: number;   // inches from right wall to classify as "near wall" (default 8)
  fieldSize?: number;    // default 144
  robotWidth?: number;   // default 18
  robotHeight?: number;  // default 18
  ballRadius?: number;   // default 2.5
  obstacles?: ObstaclePolygon[];
}

export function generateFARPath(
  balls: BallPosition[],
  options: FARPathOptions = {},
): OptimalPathResult {
  const {
    wallMargin = 8,
    fieldSize: fs = 144,
    robotWidth: rw = 18,
    robotHeight: rh = 18,
    ballRadius = 2.5,
    obstacles = [],
  } = options;

  const io = rh / 2 + ballRadius; // intake offset
  const color = getRandomColor();
  const startPose = FAR_START_RED;
  const startPos = { x: startPose.x, y: startPose.y };
  const startPoint = poseToPoint(startPose);

  // Sort balls: ball1=top (highest Y), ball2=middle, ball3=bottom (lowest Y)
  const sorted = [...balls].sort((a, b) => b.y - a.y);
  if (sorted.length < 3) {
    // Fewer than 3 balls — fall back to simple curved segments
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    let cur = startPos, curUx = 1, curUy = 0;
    for (const ball of sorted) {
      const dx = ball.x - cur.x, dy = ball.y - cur.y, d = vecLen(dx, dy);
      const ux = dx/d, uy = dy/d;
      const offset = Math.max(io - 1, rh/2 - 1);
      const rc = clampToField({ x: ball.x - ux*offset, y: ball.y - uy*offset }, rw, rh, fs);
      const seg = makeStraight(tangentialPoint(rc), `Intake ${ball.id}`, color);
      lines.push(seg); sequence.push({ kind: "path", lineId: seg.id! });
      cur = rc; curUx = ux; curUy = uy;
    }
    return { startPoint, lines, sequence };
  }

  const [b1, b2, b3] = sorted; // b1=top, b2=middle, b3=bottom
  const nearWall = (b: BallPosition) => b.x >= fs - wallMargin;
  const wallX = fs - io; // robot center X when flush against right wall

  // ── Helpers ────────────────────────────────────────────────────────────────

  function wallRC(b: BallPosition) {
    return clampToField({ x: wallX, y: b.y }, rw, rh, fs);
  }

  function freeRC(from: { x: number; y: number }, ball: BallPosition, offsetScale = 1) {
    const dx = ball.x - from.x, dy = ball.y - from.y, d = vecLen(dx, dy);
    const offset = (io - 1) * offsetScale;
    return clampToField({ x: ball.x - (dx/d)*offset, y: ball.y - (dy/d)*offset }, rw, rh, fs);
  }

  function bezierLine(
    from: { x: number; y: number }, fromUx: number, fromUy: number,
    to: { x: number; y: number }, toUx: number, toUy: number,
    startDeg: number, endDeg: number, name: string, tDep?: number, tArr?: number,
  ): Line {
    const chord = vecLen(to.x - from.x, to.y - from.y);
    const td = tDep ?? clamp(chord * 0.38, 6, 50);
    const ta = tArr ?? clamp(chord * 0.55, 8, 60);
    const cp1 = { x: from.x + fromUx*td, y: from.y + fromUy*td };
    const cp2 = { x: to.x   - toUx*ta,   y: to.y   - toUy*ta   };
    return {
      id: `far-${Math.random().toString(36).slice(2,9)}`,
      name,
      endPoint: linearPoint(to, startDeg, endDeg),
      controlPoints: [cp1, cp2],
      color, locked: false, waitBeforeMs: 0, waitAfterMs: 0, waitBeforeName: "", waitAfterName: "",
    };
  }

  function straightLine(to: { x: number; y: number }, startDeg: number, endDeg: number, name: string): Line {
    return {
      id: `far-${Math.random().toString(36).slice(2,9)}`,
      name,
      endPoint: linearPoint(to, startDeg, endDeg),
      controlPoints: [],
      color, locked: false, waitBeforeMs: 0, waitAfterMs: 0, waitBeforeName: "", waitAfterName: "",
    };
  }

  function seq(lines: Line[]): SequenceItem[] {
    return lines.map(l => ({ kind: "path" as const, lineId: l.id! }));
  }

  function angleDeg(from: { x: number; y: number }, to: { x: number; y: number }): number {
    return Math.atan2(to.y - from.y, to.x - from.x) * 180 / Math.PI;
  }

  // ── Classify case ──────────────────────────────────────────────────────────
  const b1wall = nearWall(b1), b2wall = nearWall(b2), b3wall = nearWall(b3);

  // ── CASE 1: All 3 near wall ────────────────────────────────────────────────
  // Start at top ball (b1), strafe down fast with quickly opening heading.
  // The heading opens from 0° (facing wall) toward -90° (facing down) quickly
  // so the intake sweeps b2 and b3 as the robot slides down the wall.
  if (b1wall && b2wall && b3wall) {
    const rc1 = wallRC(b1);
    const rc3 = wallRC(b3);

    // Approach b1 from startPos
    const startDeg = angleDeg(startPos, rc1);
    const approach = bezierLine(
      startPos, 1, 0,          // depart east
      rc1, 1, 0,               // arrive facing wall (east)
      startDeg, 0,
      "FAR Approach",
    );

    // Strafe down from b1 to b3 — heading opens from 0° to point toward b3
    // Use a steep final angle so the robot is nearly facing down by b3
    const strafeFinalDeg = angleDeg(rc1, b3) - 15; // slightly past b3 direction
    const strafe = straightLine(rc3, 0, clamp(strafeFinalDeg, -80, -20), "FAR Strafe");

    const lines = [approach, strafe];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── CASE 2: All 3 far from wall ───────────────────────────────────────────
  // One spline from below: start → approach below b3 → through b3 → b2 → b1.
  // Check if there's room to come from below b3 (b3.y > startPos.y + robot clearance).
  if (!b1wall && !b2wall && !b3wall) {
    const clearance = rh + 4; // need at least one robot height + 4" below b3
    const canApproachFromBelow = b3.y > startPos.y + clearance;

    // RC for each ball approached from below (or from the side if no room)
    let rc3: { x: number; y: number }, rc2: { x: number; y: number }, rc1: { x: number; y: number };

    if (canApproachFromBelow) {
      // Approach straight up — place RCs directly below each ball
      rc3 = clampToField({ x: b3.x, y: b3.y - (io - 1) }, rw, rh, fs);
      rc2 = clampToField({ x: b2.x, y: b2.y - (io - 1) }, rw, rh, fs);
      rc1 = clampToField({ x: b1.x, y: b1.y - (io - 1) }, rw, rh, fs);
    } else {
      // Not enough room below — approach from the left (from startPos direction)
      rc3 = freeRC(startPos, b3);
      rc2 = freeRC(rc3, b2);
      rc1 = freeRC(rc2, b1);
    }

    // Check for obstacle collisions along the spline — if any RC hits an obstacle,
    // nudge it left
    const rcs = [rc3, rc2, rc1].map(rc => {
      if (obstacles.length > 0 && robotHitsObstacle(rc.x, rc.y, rw/2, rh/2, obstacles)) {
        return clampToField({ x: rc.x - rw, y: rc.y }, rw, rh, fs);
      }
      return rc;
    });

    const splineLines = buildBezierChain(startPos, rcs, "FAR Spline", color, rw, rh, fs, obstacles, true);
    return { startPoint, lines: splineLines, sequence: seq(splineLines) };
  }

  // ── CASE 3: Ball1 near wall, ball2+ball3 far ───────────────────────────────
  // Approach b3 from below-left, intake angled upper-right (toward b1).
  // Sweep through b2 with heading rotating toward wall.
  // Hook sharply into b1 (near wall) — "hockey stick".
  if (b1wall && !b2wall && !b3wall) {
    // Approach angle to b3: split between "from below" and "toward b1"
    // The heading at b3 should face roughly toward b1 (upper-right).
    const toB1Deg = angleDeg(b3, b1);   // direction from b3 toward b1
    const fromBelowDeg = 90;            // straight up
    // Blend: 60% toward b1, 40% from below — intake opens toward b1 on arrival
    const b3ArrivalDeg = toB1Deg * 0.6 + fromBelowDeg * 0.4;
    const b3ArrRad = b3ArrivalDeg * Math.PI / 180;
    const b3ArrUx = Math.cos(b3ArrRad), b3ArrUy = Math.sin(b3ArrRad);

    // RC at b3: offset backward along arrival direction
    const rc3 = clampToField(
      { x: b3.x - b3ArrUx*(io-1), y: b3.y - b3ArrUy*(io-1) }, rw, rh, fs);

    // Depart start heading eastward
    const startToDeg = angleDeg(startPos, rc3);
    const approach = bezierLine(
      startPos, 1, 0,
      rc3, b3ArrUx, b3ArrUy,
      startToDeg, b3ArrivalDeg,
      "FAR Approach b3",
    );

    // From b3 RC, sweep to b2 — heading continuing to open toward wall
    const toWallDeg = 0; // facing east (right wall)
    // Intermediate heading at b2: between b3 arrival and wall-facing
    const b2HeadingDeg = b3ArrivalDeg * 0.4 + toWallDeg * 0.6;
    const b2HeadRad = b2HeadingDeg * Math.PI / 180;
    const b2HeadUx = Math.cos(b2HeadRad), b2HeadUy = Math.sin(b2HeadRad);
    const rc2 = clampToField(
      { x: b2.x - b2HeadUx*(io-1), y: b2.y - b2HeadUy*(io-1) }, rw, rh, fs);

    const sweep = bezierLine(
      rc3, b3ArrUx, b3ArrUy,
      rc2, b2HeadUx, b2HeadUy,
      b3ArrivalDeg, b2HeadingDeg,
      "FAR Sweep b2",
    );

    // Hook into b1: sharp curve from b2 RC heading fully into the wall
    const rc1 = wallRC(b1);
    const hookChord = vecLen(rc1.x - rc2.x, rc1.y - rc2.y);
    // Depart b2 heading somewhat toward b1, arrive at b1 facing east
    const hook = bezierLine(
      rc2, b2HeadUx, b2HeadUy,
      rc1, 1, 0,
      b2HeadingDeg, 0,
      "FAR Hook b1",
      clamp(hookChord * 0.25, 4, 20), // short departure
      clamp(hookChord * 0.7, 8, 40),  // long arrival tension = sharp hook
    );

    const lines = [approach, sweep, hook];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── CASE 2.5: Ball3 near wall, ball1+ball2 far ─────────────────────────────
  // Mirror of Case 3: approach b1 from below, sweep through b2, hook into b3.
  if (!b1wall && !b2wall && b3wall) {
    const toB3Deg = angleDeg(b1, b3);   // direction from b1 toward b3 (lower-right)
    const fromBelowDeg = 90;
    const b1ArrivalDeg = toB3Deg * 0.6 + fromBelowDeg * 0.4;
    const b1ArrRad = b1ArrivalDeg * Math.PI / 180;
    const b1ArrUx = Math.cos(b1ArrRad), b1ArrUy = Math.sin(b1ArrRad);

    const rc1 = clampToField(
      { x: b1.x - b1ArrUx*(io-1), y: b1.y - b1ArrUy*(io-1) }, rw, rh, fs);

    const startToDeg = angleDeg(startPos, rc1);
    const approach = bezierLine(
      startPos, 1, 0,
      rc1, b1ArrUx, b1ArrUy,
      startToDeg, b1ArrivalDeg,
      "FAR Approach b1",
    );

    const toWallDeg = 0;
    const b2HeadingDeg = b1ArrivalDeg * 0.4 + toWallDeg * 0.6;
    const b2HeadRad = b2HeadingDeg * Math.PI / 180;
    const b2HeadUx = Math.cos(b2HeadRad), b2HeadUy = Math.sin(b2HeadRad);
    const rc2 = clampToField(
      { x: b2.x - b2HeadUx*(io-1), y: b2.y - b2HeadUy*(io-1) }, rw, rh, fs);

    const sweep = bezierLine(
      rc1, b1ArrUx, b1ArrUy,
      rc2, b2HeadUx, b2HeadUy,
      b1ArrivalDeg, b2HeadingDeg,
      "FAR Sweep b2",
    );

    const rc3 = wallRC(b3);
    const hookChord = vecLen(rc3.x - rc2.x, rc3.y - rc2.y);
    const hook = bezierLine(
      rc2, b2HeadUx, b2HeadUy,
      rc3, 1, 0,
      b2HeadingDeg, 0,
      "FAR Hook b3",
      clamp(hookChord * 0.25, 4, 20),
      clamp(hookChord * 0.7, 8, 40),
    );

    const lines = [approach, sweep, hook];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── CASE 4: Ball1+ball3 near wall, ball2 far (triangle) ───────────────────
  // Approach b2 from start, curve to b1 (top wall), strafe down to b3 (bottom wall).
  if (b1wall && !b2wall && b3wall) {
    // Approach b2 from startPos — come from below-left
    const rc2 = freeRC(startPos, b2);
    const b2Deg = angleDeg(startPos, rc2);
    const b2ArrRad = b2Deg * Math.PI / 180;
    const b2ArrUx = Math.cos(b2ArrRad), b2ArrUy = Math.sin(b2ArrRad);

    const approach = bezierLine(
      startPos, 1, 0,
      rc2, b2ArrUx, b2ArrUy,
      angleDeg(startPos, rc2), b2Deg,
      "FAR Approach b2",
    );

    // Curve from b2 to b1 (top wall ball)
    const rc1 = wallRC(b1);
    const b1Deg = 0; // facing wall on arrival
    const toB1Chord = vecLen(rc1.x - rc2.x, rc1.y - rc2.y);
    const curve = bezierLine(
      rc2, b2ArrUx, b2ArrUy,
      rc1, 1, 0,
      b2Deg, b1Deg,
      "FAR Curve b1",
      clamp(toB1Chord * 0.3, 4, 25),
      clamp(toB1Chord * 0.6, 8, 40),
    );

    // Strafe down from b1 to b3 with opening heading
    const rc3 = wallRC(b3);
    const strafeFinalDeg = angleDeg(rc1, b3) - 10;
    const strafe = straightLine(rc3, 0, clamp(strafeFinalDeg, -75, -15), "FAR Strafe b3");

    const lines = [approach, curve, strafe];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── CASE 5: Ball1+ball2 near wall, ball3 far ─────────────────────────────
  // Approach b3 from below, then curve to b2 (lower wall ball), strafe up to b1.
  if (b1wall && b2wall && !b3wall) {
    const rc3 = freeRC(startPos, b3);
    const b3Deg = angleDeg(startPos, rc3);
    const b3Rad = b3Deg * Math.PI / 180;
    const b3Ux = Math.cos(b3Rad), b3Uy = Math.sin(b3Rad);

    const approach = bezierLine(
      startPos, 1, 0, rc3, b3Ux, b3Uy,
      angleDeg(startPos, rc3), b3Deg, "FAR Approach b3",
    );

    const rc2 = wallRC(b2);
    const toB2Chord = vecLen(rc2.x - rc3.x, rc2.y - rc3.y);
    const curve = bezierLine(
      rc3, b3Ux, b3Uy, rc2, 1, 0,
      b3Deg, 0, "FAR Curve b2",
      clamp(toB2Chord * 0.3, 4, 25),
      clamp(toB2Chord * 0.6, 8, 40),
    );

    const rc1 = wallRC(b1);
    const strafeFinalDeg = angleDeg(rc2, b1) + 10;
    const strafe = straightLine(rc1, 0, clamp(strafeFinalDeg, 15, 75), "FAR Strafe b1");

    const lines = [approach, curve, strafe];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── CASE 6: Ball2+ball3 near wall, ball1 far ──────────────────────────────
  // Approach b1 from below, curve to b3 (lower wall ball), strafe up to b2.
  if (!b1wall && b2wall && b3wall) {
    const rc1 = freeRC(startPos, b1);
    const b1Deg = angleDeg(startPos, rc1);
    const b1Rad = b1Deg * Math.PI / 180;
    const b1Ux = Math.cos(b1Rad), b1Uy = Math.sin(b1Rad);

    const approach = bezierLine(
      startPos, 1, 0, rc1, b1Ux, b1Uy,
      angleDeg(startPos, rc1), b1Deg, "FAR Approach b1",
    );

    const rc3 = wallRC(b3);
    const toB3Chord = vecLen(rc3.x - rc1.x, rc3.y - rc1.y);
    const curve = bezierLine(
      rc1, b1Ux, b1Uy, rc3, 1, 0,
      b1Deg, 0, "FAR Curve b3",
      clamp(toB3Chord * 0.3, 4, 25),
      clamp(toB3Chord * 0.6, 8, 40),
    );

    const rc2 = wallRC(b2);
    const strafeFinalDeg = angleDeg(rc3, b2) + 10;
    const strafe = straightLine(rc2, 0, clamp(strafeFinalDeg, 15, 75), "FAR Strafe b2");

    const lines = [approach, curve, strafe];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── CASE 7: Only ball2 near wall, ball1+ball3 far ─────────────────────────
  // Approach b3 from below, curve up through b2 (wall), continue to b1.
  if (!b1wall && b2wall && !b3wall) {
    const rc3 = freeRC(startPos, b3);
    const b3Deg = angleDeg(startPos, rc3);
    const b3Rad = b3Deg * Math.PI / 180;
    const b3Ux = Math.cos(b3Rad), b3Uy = Math.sin(b3Rad);

    const approach = bezierLine(
      startPos, 1, 0, rc3, b3Ux, b3Uy,
      angleDeg(startPos, rc3), b3Deg, "FAR Approach b3",
    );

    const rc2 = wallRC(b2);
    const rc2Deg = 0;
    const toB2Chord = vecLen(rc2.x - rc3.x, rc2.y - rc3.y);
    const seg2 = bezierLine(
      rc3, b3Ux, b3Uy, rc2, 1, 0,
      b3Deg, rc2Deg, "FAR to b2",
      clamp(toB2Chord * 0.3, 4, 25),
      clamp(toB2Chord * 0.55, 8, 40),
    );

    const rc1 = freeRC(rc2, b1);
    const toB1Dx = b1.x - rc2.x, toB1Dy = b1.y - rc2.y, toB1D = vecLen(toB1Dx, toB1Dy);
    const b1Ux = toB1Dx/toB1D, b1Uy = toB1Dy/toB1D;
    const seg3 = bezierLine(
      rc2, 1, 0, rc1, b1Ux, b1Uy,
      rc2Deg, angleDeg(rc2, rc1), "FAR to b1",
    );

    const lines = [approach, seg2, seg3];
    return { startPoint, lines, sequence: seq(lines) };
  }

  // ── FALLBACK: anything else — three separate curved segments ─────────────
  {
    const lines: Line[] = [], sequence: SequenceItem[] = [];
    let cur = startPos, curUx = 1, curUy = 0;
    for (const ball of [b3, b2, b1]) { // approach bottom-to-top
      const dx = ball.x - cur.x, dy = ball.y - cur.y, d = vecLen(dx, dy);
      const ux = dx/d, uy = dy/d;
      const rc = clampToField({ x: ball.x - ux*(io-1), y: ball.y - uy*(io-1) }, rw, rh, fs);
      const dep = clamp(vecLen(rc.x-cur.x, rc.y-cur.y)*0.38, 6, 50);
      const arr = clamp(vecLen(rc.x-cur.x, rc.y-cur.y)*0.55, 8, 60);
      const cp1 = { x: cur.x + curUx*dep, y: cur.y + curUy*dep };
      const cp2 = { x: rc.x - ux*arr,     y: rc.y - uy*arr     };
      const startDeg = Math.atan2(curUy, curUx) * 180/Math.PI;
      const endDeg   = Math.atan2(uy, ux) * 180/Math.PI;
      const seg: Line = {
        id: `far-${Math.random().toString(36).slice(2,9)}`,
        name: `Intake ${ball.id}`,
        endPoint: linearPoint(rc, startDeg, endDeg),
        controlPoints: [cp1, cp2],
        color, locked:false, waitBeforeMs:0, waitAfterMs:0, waitBeforeName:"", waitAfterName:"",
      };
      lines.push(seg); sequence.push({ kind:"path", lineId:seg.id! });
      cur = rc; curUx = ux; curUy = uy;
    }
    return { startPoint, lines, sequence };
  }
}
