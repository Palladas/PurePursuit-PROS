#include "main.h"
#include <cmath>
#include <iostream>

// class to handle angles
class angle {
public:
  double m_v; // value in radians
  double m_sin;
  double m_cos;

  angle() {
    m_v = 0.0;
    m_cos = 1.0;
    m_sin = 0.0;
  }

  // value in radians
  angle(double value) {
    m_v = value;
    m_sin = sin(value);
    m_cos = cos(value);
  }
  angle(double x, double y) {
    double magnitude = std::hypot(x, y);
    if (magnitude > 1e-6) {
      m_sin = y / magnitude;
      m_cos = x / magnitude;
    } else {
      m_sin = 0.0;
      m_cos = 1.0;
    }
    m_v = atan2(m_sin, m_cos);
  }

  angle plus(angle other) { return rotateBy(other); }

  angle minus(angle other) { return rotateBy(other.neg()); }

  angle neg() { return angle(-m_v); }

  angle rotateBy(angle other) {
    return angle(m_cos * other.m_cos - m_sin * other.m_sin,
                 m_cos * other.m_sin + m_sin * other.m_cos);
  }
};

// class to handle 1-D distances
class dist {
public:
  double m_x;
  double m_y;

  dist() {
    m_x = 0.0;
    m_y = 0.0;
  }

  dist(double x, double y) {
    m_x = x;
    m_y = y;
  }

  dist(double distance, angle omega) {
    m_x = distance * omega.m_cos;
    m_y = distance * omega.m_sin;
  }

  double getDistance(dist other) {
    return hypot(other.m_x - m_x, other.m_y - m_y);
  }

  double getNorm() { return hypot(m_x, m_y); }

  angle getAngle() { return angle(m_x, m_y); }

  dist rotateBy(angle other) {
    return dist(m_x * other.m_cos - m_y * other.m_sin,
                m_x * other.m_sin + m_y * other.m_cos);
  }

  dist plus(dist other) { return dist(m_x + other.m_x, m_y + other.m_y); }

  dist minus(dist other) { return dist(m_x - other.m_x, m_y - other.m_y); }
};

// class to handle deltas

class delta {
public:
  // linear axial components, angular
  double dx;
  double dy;
  double dt;
  delta() {}

  delta(double x, double y, double t) {
    dx = x;
    dy = y;
    dt = t;
  }
};

class transformation {
public:
  dist m_d;
  angle m_r;
  transformation(dist initial_mp, angle initial_mr, dist last_mp,
                 angle last_mr) {
    m_d = last_mp.minus(initial_mp).rotateBy(initial_mr.neg());
    m_r = last_mr.minus(initial_mr);
  }

  transformation(dist translation, angle rotation) {
    m_d = translation;
    m_r = rotation;
  }
};

std::ostream &operator<<(std::ostream &os, const angle &x) {
  os << " Angle: " << x.m_v << " Cos: " << x.m_cos << " Sin: " << x.m_sin;
  return os;
}

std::ostream &operator<<(std::ostream &os, const dist &x) {
  os << " X: " << x.m_x << " Y: " << x.m_y;
  return os;
}

std::ostream &operator<<(std::ostream &os, const delta &x) {
  os << " X: " << x.dx << " Y: " << x.dy << " Theta: " << x.dt;
  return os;
}

std::ostream &operator<<(std::ostream &os, const transformation &x) {
  os << " Linear: " << x.m_d << " Rotational: " << x.m_r;
  return os;
}

// class that has a magnitude (from the origin) and a rotation
// can be used to represent robot position
class point {
public:
  //----------------------------------

  dist m_p;
  angle m_r;

  point() {
    m_p = dist();
    m_r = angle();
  }

  point(dist position, angle rotation) {
    m_p = position;
    m_r = rotation;
  }

  point(double x, double y, angle rotation) {
    m_p = dist(x, y);
    m_r = rotation;
  }

  point transformBy(transformation other) {
    return point(m_p.plus(other.m_d.rotateBy(m_r)), other.m_r.plus(m_r));
  }

  point plus(transformation other) { return transformBy(other); }

  point exp(delta d) {
    double dx = d.dx;
    double dy = d.dy;
    double dt = d.dt;

    double sint = sin(dt);
    double cost = cos(dt);

    double s;
    double c;
    if (abs(dt) < 1e9) {
      s = 1.0 - 1.0 / 6.0 * dt * dt;
      c = 0.5 * dt;
    } else {
      s = sint / dt;
      c = (1 - cost) / dt;
    }

    transformation transform = transformation(
        dist(dx * s - dy * c, dx * c + dy * s), angle(cost, sint));
    std::cout << transform << '\n';
    return this->plus(transform);
  }
};

// omg debugging???

std::ostream &operator<<(std::ostream &os, const point &x) {
  os << " Pos: " << x.m_p << " Heading: " << x.m_r;
  return os;
}
