#pragma once
#include <cmath>
#include <vector>

namespace basic_control {

const double EPS = 1e-10;

namespace geometry {
// Return -1 if the value is smaller than 0.
// Return 0 if the value is equal to 0.
// Return 1 if the value is bigger than 0.
inline int DoubleCmp(double x) {
  if (std::fabs(x) < EPS) {
    return 0;
  } else {
    return x < 0 ? -1 : 1;
  }
}

struct Point2 {
  double x, y;

  Point2() = default;

  Point2(double x, double y) : x(x), y(y){};
};

typedef Point2 Vector2;

inline Vector2 operator+(const Vector2 &a, const Vector2 &b) {
  return Vector2(a.x + b.x, a.y + b.y);
}

inline Vector2 operator-(const Vector2 &a, const Vector2 &b) {
  return Vector2(a.x - b.x, a.y - b.y);
}

inline Vector2 operator*(const Vector2 &v, double s) {
  return Vector2(v.x * s, v.y * s);
}

inline Vector2 operator/(const Vector2 &v, double s) {
  return Vector2(v.x / s, v.y / s);
}

inline bool operator==(const Point2 &a, const Point2 &b) {
  return DoubleCmp(a.x - b.x) == 0 && DoubleCmp(a.y - b.y) == 0;
}

inline bool operator!=(const Point2 &a, const Point2 &b) {
  return !(DoubleCmp(b.x - a.x) == 0 && DoubleCmp(b.y - a.y) == 0);
}

inline double DotProduct(const Vector2 &a, const Vector2 &b) {
  return a.x * b.x + a.y * b.y;
}

// Sign of the product represents the relative angle between a and b.
inline double CrossProduct(const Vector2 &a, const Vector2 &b) {
  return a.x * b.y - a.y * b.x;
}

inline double LengthOfVector2(const Vector2 &a) {
  return sqrt(DotProduct(a, a));
}

// A line is represented by p + tv.
struct Line2 {
  // Direction vector.
  Vector2 v;
  Point2 p;

  // The angle from positive x axis.
  // Positive for rotating counter-clockwise.
  // Negative for rotating clockwise.
  double angle;

  Line2() = default;

  Line2(Vector2 v, Point2 p) : v(v), p(p) {
    angle = atan2(v.y, v.x);
  }

  Line2(double angle, Point2 p) : angle(angle), p(p) {
    v.x = cos(angle);
    v.y = sin(angle);
  }

  // Get the point at p + tv.
  Point2 GetPoint(double t) const {
    return p + v * t;
  };

  Vector2 GetDirection() const {
    return v;
  };

  Vector2 GetNormalDirection() const {
    return Vector2(-v.y, v.x);
  };
};

// Get the projection of point p on line l.
inline Point2 GetProjection(const Point2 &p, const Line2 &l) {
  const Vector2 &v = l.v;
  return l.p + v * (DotProduct(v, p - l.p) / DotProduct(v, v));
}

// Get the projection of point p on the line defined by a and b.
inline Point2 GetProjection(const Point2 &p, const Point2 &a, const Point2 &b) {
  Vector2 v = b - a;
  return a + v * (DotProduct(v, p - a) / DotProduct(v, v));
}

// Counter clockwise. A negative rad means clockwise.
inline Vector2 RotateVector2(const Vector2 &a, double rad) {
  return Vector2(a.x * cos(rad) - a.y * sin(rad), a.x * sin(rad) + a.y * cos(rad));
}


struct LineSegs {
  LineSegs() = default;

  std::vector<Point2> segs;

  Point2 ExtendFromIndex(int index, double dist) const {
    // TODO: this is a very coarse implementation, calls for improvement.
    for (int i = index; i < segs.size() - 1; i++) {
      if (dist <= 0) {
        return segs[i];
      } else {
        dist -= LengthOfVector2(segs[i + 1] - segs[i]);
      }
    }
    return segs.back();
  }
};

// Distance from p to the line segment (a, b).
inline double DistanceToSegment(const Point2 &p, const Point2 &a, const Point2 &b) {
  // Depends on the equal comparator defined above.
  if (a == b) {
    return LengthOfVector2(p - a);
  }
  Vector2 v1 = b - a, v2 = p - a, v3 = p - b;
  if (DoubleCmp(DotProduct(v1, v2)) < 0) {
    return LengthOfVector2(v2);
  } else if (DoubleCmp(DotProduct(v1, v3)) > 0) {
    return LengthOfVector2(v3);
  } else {
    return std::fabs(CrossProduct(v1, v2) / LengthOfVector2(v1));
  }
}

// Angle related.
// Adapted from http://docs.ros.org/en/indigo/api/angles/html/angles_8h_source.html.
inline double FromDegrees(double degrees) {
  return degrees * M_PI / 180.0;
}
inline double ToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}
inline double NormalizeAnglePositive(double angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

inline double NormalizeAngle(double angle) {
  double a = NormalizeAnglePositive(angle);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}
inline double ShortestAngularDistance(double from, double to) {
  return NormalizeAngle(to - from);
}

// This gives an undirected angle. [0, pi].
double AngleBetweenVector2(const Vector2 &a, const Vector2 &b) {
  return acos(DotProduct(a, b) / LengthOfVector2(a) / LengthOfVector2(b));
}

}  // namespace geometry

}  // namespace basic_control