
const { Vehicle } = require('../src/physics/vehicle.js');
const { resolveVehicleBarrier, resolveVehicleVehicle } = require('../src/physics/collision.js');
const { Barrier } = require('../src/track/barrier.js');

describe('赛车物理碰撞', () => {
  test('高速碰撞不应穿透', () => {
    const v = new Vehicle(100, 0);
    v.vy = 200;
    const barrier = new Barrier(0, 100, 200, 10);
    v.update(1/60);
    resolveVehicleBarrier(v, barrier);
    expect(v.y + v.height).toBeLessThanOrEqual(barrier.y + 1);
  });

  test('碰撞后位置应分离', () => {
    const v = new Vehicle(100, 95);
    v.vy = 10;
    const barrier = new Barrier(0, 100, 200, 10);
    resolveVehicleBarrier(v, barrier);
    expect(v.y + v.height).toBeLessThanOrEqual(barrier.y);
  });

  test('车辆间碰撞应双向修正', () => {
    const v1 = new Vehicle(90, 50);
    const v2 = new Vehicle(100, 50);
    v1.vx = 10; v2.vx = 0;
    resolveVehicleVehicle(v1, v2);
    expect(v1.vx).toBeLessThan(10);
    expect(v2.vx).toBeGreaterThan(0);
  });

  test('恢复系数应应用', () => {
    const v = new Vehicle(100, 0);
    v.vy = 10;
    const barrier = new Barrier(0, 50, 200, 10);
    resolveVehicleBarrier(v, barrier);
    expect(v.vy).toBeLessThanOrEqual(0);
  });

  test('静止车辆被撞后应移动', () => {
    const v1 = new Vehicle(80, 50);
    const v2 = new Vehicle(100, 50);
    v1.vx = 20; v2.vx = 0;
    resolveVehicleVehicle(v1, v2);
    expect(v2.vx).toBeGreaterThan(0);
  });

  test('斜角碰撞响应', () => {
    const v = new Vehicle(100, 0);
    v.vx = 5; v.vy = 10;
    const barrier = new Barrier(0, 50, 200, 10);
    resolveVehicleBarrier(v, barrier);
    expect(v.vx).toBe(5); // 切线速度保留
  });
});
