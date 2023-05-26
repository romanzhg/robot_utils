#pragma once

#include "constants.h"

#include <cmath>
#include <climits>
#include <cfloat>
#include <vector>
#include <cassert>

namespace basic_control {

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
  Point2(const Point2& o) = default;

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

  int GetClosestPointIndex(double x, double y) const {
    geometry::Point2 p(x, y);
    int segs_size = segs.size();
    
    double min_dist = DBL_MAX / 2;
    int min_index = 0;
    for (int i = 0; i < segs_size; i++) {
      const geometry::Point2& a = segs[i];
      double tmp_dist = geometry::LengthOfVector2(p - a);
      if (min_dist > tmp_dist) {
        min_dist = tmp_dist;
        min_index = i;
      }
    }
    return min_index;
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
inline double AngleBetweenVector2(const Vector2 &a, const Vector2 &b) {
  return acos(DotProduct(a, b) / LengthOfVector2(a) / LengthOfVector2(b));
}

inline double GetDist(Point2 a, Point2 b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

inline double TriangleArea(Point2 a, Point2 b, Point2 c) {
  double lab = GetDist(a, b);
  double lac = GetDist(a, c);
  double lbc = GetDist(b, c);
  double s = (lab + lac + lbc) / 2.0;
  return std::sqrt(s * (s - lab) * (s - lac) * (s - lbc));
}

// Calculates 3-points curvature.
// https://en.wikipedia.org/wiki/Menger_curvature
inline double GetCurvature(Point2 a, Point2 b, Point2 c) {
  double area = TriangleArea(a, b, c);
  double lab = GetDist(a, b);
  double lac = GetDist(a, c);
  double lbc = GetDist(b, c);
  return 4 * area / (lab * lac * lbc);
}

// For two vectors a->p, a->b, if the rotation(counter-clockwise) from a->p to a->b
// is in [0, PI], return 1, else if the rotation is in [-PI, 0) return -1.
inline double GetRotationSign(const Point2& p, const Point2& a, const Point2& b) {
  Vector2 v_from = p - a, v_to = b - a;
  double v = CrossProduct(v_from, v_to);
  return v > 0 ? 1.0 : -1.0;
}

// The returned value is positive if (x, y) is on the left of the reference line.
inline double GetCrossTrackError(double x, double y, const LineSegs& ref) {
  Point2 p(x, y);
  int size = ref.segs.size();
  if (size < 2) {
    // TODO: print the line number here.
    exit(0);
  }
  
  double xte = DBL_MAX / 2;
  double sign = 1.0;
  for (int i = 0; i < size - 1; i++) {
    const Point2& a = ref.segs[i];
    const Point2& b = ref.segs[i + 1];
    double tmp_dist = DistanceToSegment(p, a, b);
    if (xte > tmp_dist) {
      xte = tmp_dist;
      sign = -1.0 * GetRotationSign(p, a, b);
    }
  }
  return xte * sign;
}

inline double GetRotationValueHelper(double from_yaw, double to_yaw) {
    double angle_if_rotate_left = std::fmod(to_yaw - from_yaw, 2 * M_PI);
    double angle_if_rotate_right = 2 * M_PI - angle_if_rotate_left;

    if (angle_if_rotate_left <= angle_if_rotate_right) {
        return angle_if_rotate_left;
    } else {
        return -angle_if_rotate_right;
    }
}

inline double GetRotationValue(double from_yaw, double to_yaw) {
  if (from_yaw < to_yaw) {
    return GetRotationValueHelper(from_yaw, to_yaw);
  } else {
    return -GetRotationValueHelper(to_yaw, from_yaw);
  }
}


}  // namespace geometry

}  // namespace basic_control