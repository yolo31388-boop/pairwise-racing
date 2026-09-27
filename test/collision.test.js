const { Vehicle } = require('../src/physics/vehicle.js');
const { resolveVehicleBarrier, resolveVehicleVehicle, sweptAABB, overlaps } = require('../src/physics/collision.js');
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

describe('冲量 / 法线 / CCD 强化测试', () => {
  test('高速CCD:一帧速度足以跨越薄护栏也不会穿透', () => {
    const v = new Vehicle(100, 50);   // 底边 90
    v.vy = 2000;                      // 一帧位移 ~33.3,整段越过护栏(100~105)
    const barrier = new Barrier(0, 100, 200, 5);
    const info = resolveVehicleBarrier(v, barrier, 1 / 60);

    expect(info.swept).toBe(true);
    expect(info.t).toBeCloseTo(0.3, 5);
    expect(v.y + v.height).toBeLessThanOrEqual(barrier.y + 1e-3); // 被挡在护栏顶面
    expect(v.vy).toBeLessThan(0);       // 反弹(冲量),而不是清零或穿出
  });

  test('sweptAABB 能识别整帧跨越(无CCD时必然隧穿)', () => {
    const v = new Vehicle(100, 50);
    const barrier = new Barrier(0, 100, 200, 5);
    // 本帧起点、终点都不与护栏重叠(经典隧穿)
    v.update(1 / 60);
    expect(overlaps(v, barrier)).toBe(false);
    expect(sweptAABB(new Vehicle(100, 50), barrier, 0, 2000 / 60)).not.toBeNull();
  });

  test('法线是碰撞点处护栏真实法线,而非中心连线', () => {
    // 高大护栏:车辆中心与护栏中心连线带 y 分量,但撞击的是护栏左侧面
    const barrier = new Barrier(100, 0, 20, 300);
    const v = new Vehicle(50, 150);
    v.vx = 31;
    const info = resolveVehicleBarrier(v, barrier);
    expect(info.normal.x).toBe(-1);
    expect(info.normal.y).toBe(0);
  });

  test('冲量响应:e=1 时法向速度等大反向(不是简单清零)', () => {
    const v = new Vehicle(100, 50);
    v.vy = 20;
    v.restitution = 1;
    const barrier = new Barrier(0, 100, 200, 10);
    barrier.restitution = 1;
    resolveVehicleBarrier(v, barrier);
    expect(v.vy).toBeCloseTo(-20, 6);
    expect(v.y + v.height).toBeCloseTo(100, 5);
  });

  test('恢复系数:e=0.3 时反弹速度为入射的0.3倍', () => {
    const v = new Vehicle(100, 50);
    v.vy = 20;
    v.restitution = 0.3;
    const barrier = new Barrier(0, 100, 200, 10);
    barrier.restitution = 0.3;
    resolveVehicleBarrier(v, barrier);
    expect(v.vy).toBeCloseTo(-6, 6);
  });

  test('斜角碰撞:法向反射、切向保留(护栏光滑)', () => {
    const v = new Vehicle(100, 50);
    v.vx = 8; v.vy = 20;
    v.restitution = 0.5;
    const barrier = new Barrier(0, 100, 200, 10);
    barrier.restitution = 0.5;
    resolveVehicleBarrier(v, barrier);
    expect(v.vy).toBeCloseTo(-10, 6);
    expect(v.vx).toBeCloseTo(8, 6);
  });

  test('碰撞后接触摩擦削减切向速度(库仑摩擦受法向冲量约束)', () => {
    const v = new Vehicle(100, 50);
    v.vx = 12; v.vy = 20;
    v.restitution = 0.5;
    v.contactFriction = 1;
    const barrier = new Barrier(0, 100, 200, 10);
    barrier.restitution = 0.5;
    barrier.friction = 1;
    resolveVehicleBarrier(v, barrier);
    expect(v.vy).toBeCloseTo(-10, 6); // 法向仍按 e 反射
    expect(Math.abs(v.vx)).toBeLessThan(1e-6); // 切向被摩擦吃掉
  });

  test('滚动阻力:松油门后速度持续衰减,不会无限滑动', () => {
    const v = new Vehicle(0, 0);
    v.vx = 10;
    v.friction = 2;
    for (let i = 0; i < 3; i++) v.update(1);
    expect(v.vx).toBeCloseTo(10 * Math.exp(-6), 5);
    for (let i = 0; i < 20; i++) v.update(1);
    expect(v.getState().vx + v.getState().vy).toBe(0); // 最终静止
  });

  test('嵌入护栏的车辆被推出到不相交状态', () => {
    const v = new Vehicle(100, 105); // 已嵌入护栏 (100~110)
    v.vy = 10;
    const barrier = new Barrier(0, 100, 200, 10);
    resolveVehicleBarrier(v, barrier);
    expect(overlaps(v, barrier)).toBe(false);
  });

  test('车辆间碰撞:速度与位置双向修正且动量守恒', () => {
    const v1 = new Vehicle(70, 50);
    const v2 = new Vehicle(100, 50);
    v1.vx = 20; v2.vx = 0;
    resolveVehicleVehicle(v1, v2);
    expect(v1.vx).toBeCloseTo(5, 6);
    expect(v2.vx).toBeCloseTo(15, 6);
    expect(v1.vx + v2.vx).toBeCloseTo(20, 6); // 动量守恒(等质量)
    expect(overlaps(v1, v2)).toBe(false);     // 位置分离
  });

  test('弹性对撞近似能量守恒(等质量 e=1 交换速度)', () => {
    const v1 = new Vehicle(70, 50);
    const v2 = new Vehicle(100, 50);
    v1.restitution = 1; v2.restitution = 1;
    v1.contactFriction = 0; v2.contactFriction = 0;
    v1.vx = 20; v2.vx = 0;
    const ke0 = 0.5 * (v1.vx * v1.vx + v2.vx * v2.vx);
    resolveVehicleVehicle(v1, v2);
    expect(v1.vx).toBeCloseTo(0, 6);
    expect(v2.vx).toBeCloseTo(20, 6);
    const ke1 = 0.5 * (v1.vx * v1.vx + v2.vx * v2.vx);
    expect(ke1).toBeCloseTo(ke0, 6);
  });

  test('重车(不可动)被撞保持静止,来车被反弹', () => {
    const v1 = new Vehicle(70, 50);
    const v2 = new Vehicle(100, 50);
    v1.vx = 20; v2.vx = 0;
    v1.restitution = 1; v2.restitution = 1;
    v1.contactFriction = 0; v2.contactFriction = 0;
    v2.setMass(Infinity);
    resolveVehicleVehicle(v1, v2);
    expect(v2.vx).toBe(0);
    expect(v1.vx).toBeCloseTo(-20, 6);
    expect(overlaps(v1, v2)).toBe(false);
  });

  test('多车连续碰撞:冲量沿车列传递,顺序与分离均正确', () => {
    const c1 = new Vehicle(60, 50);
    const c2 = new Vehicle(100, 50);
    const c3 = new Vehicle(140, 50);
    c1.vx = 30; c2.vx = 0; c3.vx = 0;
    resolveVehicleVehicle(c1, c2, 1);
    resolveVehicleVehicle(c2, c3, 1);
    expect(c3.vx).toBeGreaterThan(0);   // 末端车获得冲量
    expect(c2.vx).toBeGreaterThan(0);
    expect(overlaps(c1, c2)).toBe(false);
    expect(overlaps(c2, c3)).toBe(false);
    expect(c1.x + c1.width).toBeLessThanOrEqual(c2.x + 1e-6);
    expect(c2.x + c2.width).toBeLessThanOrEqual(c3.x + 1e-6);
  });

  test('applyForce / getState 公开用途保持一致', () => {
    const v = new Vehicle(0, 0);
    v.applyForce(10, 0, 1);
    expect(v.vx).toBe(10);
    v.setMass(2);
    v.applyForce(10, 0, 1);
    expect(v.vx).toBeCloseTo(15, 9);
    const s = v.getState();
    expect(s).toEqual(expect.objectContaining({ vx: 15, vy: 0, width: 20, height: 40 }));
  });
});
