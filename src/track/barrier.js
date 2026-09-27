// 护栏:轴对齐矩形静态体。构造签名 (x, y, w, h) 保持不变。
// 高速赛道护栏按刚体静态体处理(invMass = 0),不被推动。
class Barrier {
  constructor(x, y, w, h) {
    this.x = x;
    this.y = y;
    this.width = w;
    this.height = h;

    this.mass = Infinity;
    this.invMass = 0;
    this.restitution = 0.5; // 护栏恢复系数
    this.friction = 0;      // 护栏表面接触摩擦(默认光滑,碰撞切线速度保留)
  }

  // 返回碰撞点 px,py 处护栏的真实表面法线(单位向量,指向护栏外)。
  // 依据碰撞点落在护栏哪一面来决定,而不是用“车辆中心->护栏中心”的连线。
  surfaceNormal(px, py) {
    // 碰撞点到四个面的穿透/有符号距离
    const dLeft = px - this.x;
    const dRight = (this.x + this.width) - px;
    const dTop = py - this.y;
    const dBottom = (this.y + this.height) - py;

    const min = Math.min(dLeft, dRight, dTop, dBottom);
    if (min === dLeft) return { x: -1, y: 0 };
    if (min === dRight) return { x: 1, y: 0 };
    if (min === dTop) return { x: 0, y: -1 };
    return { x: 0, y: 1 };
  }
}

module.exports = { Barrier };
