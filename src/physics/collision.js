
// 碰撞响应 - 存在穿透和法线错误bug
export function resolveVehicleBarrier(vehicle, barrier) {
  if (aabb(vehicle, barrier)) {
    // BUG: 简单位置修正，没有冲量计算
    vehicle.y = barrier.y - vehicle.height;
    vehicle.vy = 0; // BUG: 只清零y速度，不反射
    // BUG: 法线用车辆中心到护栏中心连线
  }
}

export function resolveVehicleVehicle(v1, v2) {
  if (aabb(v1, v2)) {
    // BUG: 只修一辆车
    v1.x -= 5;
  }
}

function aabb(a, b) {
  return a.x < b.x + b.width && a.x + a.width > b.x &&
         a.y < b.y + b.height && a.y + a.height > b.y;
}

module.exports = { resolveVehicleBarrier, resolveVehicleVehicle };
