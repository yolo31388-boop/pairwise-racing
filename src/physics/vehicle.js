
class Vehicle {
  constructor(x, y) {
    this.x = x; this.y = y; this.vx = 0; this.vy = 0;
    this.width = 20; this.height = 40; this.angle = 0;
  }
  applyForce(fx, fy, dt) { this.vx += fx * dt; this.vy += fy * dt; }
  update(dt) { this.x += this.vx * dt; this.y += this.vy * dt; }
}

module.exports = { Vehicle };
