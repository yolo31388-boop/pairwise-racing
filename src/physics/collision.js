// 碰撞响应:基于冲量(impulse)的解算 + 连续碰撞检测(CCD, swept-AABB)。
//
// 不变量:
//  1) 响应必须是冲量,不能只做位置修正;
//  2) 法线取碰撞点处护栏/接触面的真实法线,不能用“中心->中心”的连线;
//  3) 法向速度按恢复系数反射,切向速度保留(受接触摩擦调制);
//  4) 高速时用 swept-AABB 求最早接触时间,避免一帧穿透薄护栏;
//  5) 车辆间碰撞同时修正两辆车的速度与位置。

const EPS = 1e-6;

function overlaps(a, b) {
  return a.x < b.x + b.width && a.x + a.width > b.x &&
         a.y < b.y + b.height && a.y + a.height > b.y;
}

function center(o) {
  return { x: o.x + o.width / 2, y: o.y + o.height / 2 };
}

function clamp(v, lo, hi) {
  return Math.max(lo, Math.min(hi, v));
}

// 求 AABB 互相穿透时的接触点与分离法线(由 b 指向 a)。
// 接触点取 a 中心钳制到 b 表面,法线取该点处 b 的真实表面法线。
function staticContact(a, b) {
  const ca = center(a);
  const px = clamp(ca.x, b.x, b.x + b.width);
  const py = clamp(ca.y, b.y, b.y + b.height);
  let nx = 0;
  let ny = 0;

  if (typeof b.surfaceNormal === 'function') {
    const n = b.surfaceNormal(px, py);
    nx = n.x; ny = n.y;
  } else {
    // b 是普通车辆(无 surfaceNormal):按穿透最浅的轴分离,
    // 穿透深度相等时沿“相对接近方向”推出。
    const overlapX = Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x);
    const overlapY = Math.min(a.y + a.height, b.y + b.height) - Math.max(a.y, b.y);
    const cb = center(b);
    const sx = ca.x < cb.x ? -1 : 1;
    const sy = ca.y < cb.y ? -1 : 1;
    const rvx = a.vx - b.vx;
    const rvy = a.vy - b.vy;
    if (overlapX < overlapY) nx = sx;
    else if (overlapY < overlapX) ny = sy;
    else if (Math.abs(rvx) > Math.abs(rvy)) nx = sx;
    else ny = sy;
  }
  return { nx, ny, px, py };
}

// swept-AABB:b 为参考系,a 以相对位移 (dx,dy) 运动,求本帧最早接触。
// 返回 { t, nx, ny } 或 null。法线由 b 指向 a。
function sweptAABB(a, b, dx, dy) {
  let tEnter = -Infinity;
  let tExit = Infinity;
  let axis = null;

  const axes = [
    { d: dx, amin: a.x, amax: a.x + a.width, bmin: b.x, bmax: b.x + b.width, name: 'x' },
    { d: dy, amin: a.y, amax: a.y + a.height, bmin: b.y, bmax: b.y + b.height, name: 'y' },
  ];

  for (const ax of axes) {
    let lo;
    let hi;
    if (Math.abs(ax.d) < EPS) {
      if (ax.amax <= ax.bmin || ax.amin >= ax.bmax) return null;
      lo = -Infinity;
      hi = Infinity;
    } else {
      const t1 = (ax.bmin - ax.amax) / ax.d;
      const t2 = (ax.bmax - ax.amin) / ax.d;
      lo = Math.min(t1, t2);
      hi = Math.max(t1, t2);
    }
    if (lo > tEnter) {
      tEnter = lo;
      axis = ax;
    }
    tExit = Math.min(tExit, hi);
  }

  if (tEnter >= tExit || tExit <= 0 || tEnter > 1) return null;

  const t = Math.max(tEnter, 0);
  let nx = 0;
  let ny = 0;
  if (axis.name === 'x') nx = axis.d > 0 ? -1 : 1;
  else ny = axis.d > 0 ? -1 : 1;
  return { t, nx, ny };
}

function geoRestitution(a, b) {
  if (typeof b.restitution === 'number' && typeof a.restitution === 'number') {
    return Math.sqrt(Math.max(a.restitution, 0) * Math.max(b.restitution, 0));
  }
  return a.restitution != null ? a.restitution : (b.restitution != null ? b.restitution : 0);
}

// 把动态车辆沿法线推离静态护栏至恰好不相交。
function separateStatic(dyn, stat, nx, ny) {
  if (nx < 0) dyn.x = stat.x - dyn.width - EPS;
  else if (nx > 0) dyn.x = stat.x + stat.width + EPS;
  else if (ny < 0) dyn.y = stat.y - dyn.height - EPS;
  else if (ny > 0) dyn.y = stat.y + stat.height + EPS;
}

// 车辆 vs 护栏(静态刚体)。dt 为本帧步长;默认 1 保持直接调用兼容。
function resolveVehicleBarrier(vehicle, barrier, dt = 1) {
  const dx = vehicle.vx * dt;
  const dy = vehicle.vy * dt;

  let nx = 0;
  let ny = 0;
  let t = 0;
  let swept = false;

  const hit = sweptAABB(vehicle, barrier, dx, dy);
  if (hit) {
    ({ t, nx, ny } = hit);
    swept = true;
  } else if (overlaps(vehicle, barrier)) {
    ({ nx, ny } = staticContact(vehicle, barrier));
  } else {
    return null;
  }

  // 位置:前进到最早接触时刻,再沿真实法线分离到不相交
  if (swept) {
    vehicle.x += dx * t;
    vehicle.y += dy * t;
  }
  separateStatic(vehicle, barrier, nx, ny);

  // 冲量响应
  const rvx = vehicle.vx;
  const rvy = vehicle.vy;
  const vn = rvx * nx + rvy * ny;
  if (vn < -EPS) {
    const e = geoRestitution(vehicle, barrier);
    const jn = -(1 + e) * vn / vehicle.invMass; // 法向冲量大小

    vehicle.vx += jn * nx * vehicle.invMass;
    vehicle.vy += jn * ny * vehicle.invMass;

    // 碰撞后沿接触面的库仑摩擦(护栏表面 × 车辆轮胎)
    const mu = Math.sqrt(
      Math.max(barrier.friction || 0, 0) *
      Math.max(vehicle.contactFriction || 0, 0)
    );
    if (mu > 0) {
      const tx = -ny;
      const ty = nx;
      const vt = vehicle.vx * tx + vehicle.vy * ty;
      let jt = -vt / vehicle.invMass;
      const jtMax = mu * jn;
      jt = clamp(jt, -jtMax, jtMax);
      vehicle.vx += jt * tx * vehicle.invMass;
      vehicle.vy += jt * ty * vehicle.invMass;
    }
  }

  return { normal: { x: nx, y: ny }, t, swept };
}

// 两辆车沿法线按逆质量分摊分离距离(静态车 invMass=0 时由另一辆全让)。
function separatePair(a, b, nx, ny) {
  let depth = 0;
  if (nx !== 0) {
    depth = Math.min(a.x + a.width, b.x + b.width) - Math.max(a.x, b.x);
  } else {
    depth = Math.min(a.y + a.height, b.y + b.height) - Math.max(a.y, b.y);
  }
  if (depth <= 0) return;
  const invSum = a.invMass + b.invMass;
  if (invSum === 0) return;
  const gapA = depth * (a.invMass / invSum) + EPS;
  const gapB = depth * (b.invMass / invSum) + EPS;
  a.x += nx * gapA;
  a.y += ny * gapA;
  b.x -= nx * gapB;
  b.y -= ny * gapB;
}

// 车辆 vs 车辆:同时修正两辆车的速度与位置。
function resolveVehicleVehicle(v1, v2, dt = 1) {
  const dx = (v1.vx - v2.vx) * dt;
  const dy = (v1.vy - v2.vy) * dt;

  let nx = 0;
  let ny = 0;
  let t = 0;
  let swept = false;

  const hit = sweptAABB(v1, v2, dx, dy);
  if (hit) {
    ({ t, nx, ny } = hit);
    swept = true;
  } else if (overlaps(v1, v2)) {
    ({ nx, ny } = staticContact(v1, v2));
  } else {
    return null;
  }

  if (swept) {
    v1.x += v1.vx * dt * t;
    v1.y += v1.vy * dt * t;
    v2.x += v2.vx * dt * t;
    v2.y += v2.vy * dt * t;
  }

  const rvx = v1.vx - v2.vx;
  const rvy = v1.vy - v2.vy;
  const vn = rvx * nx + rvy * ny;

  if (vn < -EPS) {
    const e = (v1.restitution + v2.restitution) / 2;
    const invSum = v1.invMass + v2.invMass;
    const jn = -(1 + e) * vn / invSum; // 法向冲量

    v1.vx += jn * nx * v1.invMass;
    v1.vy += jn * ny * v1.invMass;
    v2.vx -= jn * nx * v2.invMass;
    v2.vy -= jn * ny * v2.invMass;

    // 沿接触面的摩擦冲量
    const mu = Math.sqrt(
      Math.max(v1.contactFriction || 0, 0) *
      Math.max(v2.contactFriction || 0, 0)
    );
    if (mu > 0) {
      const tx = -ny;
      const ty = nx;
      const vt = (v1.vx - v2.vx) * tx + (v1.vy - v2.vy) * ty;
      let jt = -vt / invSum;
      const jtMax = mu * jn;
      jt = clamp(jt, -jtMax, jtMax);
      v1.vx += jt * tx * v1.invMass;
      v1.vy += jt * ty * v1.invMass;
      v2.vx -= jt * tx * v2.invMass;
      v2.vy -= jt * ty * v2.invMass;
    }
  }

  separatePair(v1, v2, nx, ny);

  return { normal: { x: nx, y: ny }, t, swept };
}

module.exports = {
  resolveVehicleBarrier,
  resolveVehicleVehicle,
  sweptAABB,
  overlaps,
};
