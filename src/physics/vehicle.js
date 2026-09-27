// 车辆:冲量式动力学体。
// 公开用途保持不变:update(dt) 做积分,applyForce(fx, fy, dt) 施加力,getState() 读快照。
class Vehicle {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    this.vx = 0;
    this.vy = 0;
    this.width = 20;
    this.height = 40;
    this.angle = 0;

    // 动力学参数(用于冲量计算),可由调用方按车辆调参
    this.mass = 1;
    this.invMass = 1;
    this.restitution = 0.5;   // 恢复系数
    this.friction = 0.1;      // 滚动/滑行阻力系数(每秒),防止无限滑动
    this.contactFriction = 0.3; // 接触时沿接触面的库仑摩擦
  }

  setMass(m) {
    this.mass = m > 0 ? m : Infinity;
    this.invMass = m > 0 ? 1 / m : 0;
  }

  applyForce(fx, fy, dt) {
    this.vx += (fx * this.invMass) * dt;
    this.vy += (fy * this.invMass) * dt;
  }

  applyImpulse(ix, iy) {
    this.vx += ix * this.invMass;
    this.vy += iy * this.invMass;
  }

  update(dt) {
    // 阻力(碰撞后/松油门时车速指数衰减,避免无限滑动)
    if (this.friction > 0 && dt > 0) {
      const decay = Math.exp(-this.friction * dt);
      this.vx *= decay;
      this.vy *= decay;
      // 数值上把接近静止的速度钳到 0
      if (this.vx * this.vx + this.vy * this.vy < 1e-8) {
        this.vx = 0;
        this.vy = 0;
      }
    }
    this.x += this.vx * dt;
    this.y += this.vy * dt;
  }

  getState() {
    return {
      x: this.x,
      y: this.y,
      vx: this.vx,
      vy: this.vy,
      width: this.width,
      height: this.height,
      angle: this.angle,
      mass: this.mass,
      restitution: this.restitution,
      friction: this.friction,
    };
  }
}

module.exports = { Vehicle };
