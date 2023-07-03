#pragma once

#include <cmath>

namespace basic_perception {
const double EPS = 1e-8;
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

struct Point3 {
	double x, y, z;
	Point3(double x = 0, double y = 0, double z = 0): x(x), y(y), z(z) {};

	void operator=(const Point3& o) {
		x = o.x;
		y = o.y;
		z = o.z;
	}
  bool operator ==(const Point3& o) {
		return (DoubleCmp(x - o.x) == 0) && (DoubleCmp(y - o.y) == 0) && (DoubleCmp(z - o.z) == 0);
	}

	std::string ToString() {
		return std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z);
	}
};

typedef Point3 Vector3;

Vector3 operator + (const Vector3& a, const Vector3& b) {
	return Vector3(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vector3 operator - (const Vector3& a, const Vector3& b) {
	return Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector3 operator * (const Vector3& a, double p) {
	return Vector3(a.x * p, a.y * p, a.z * p);
}

Vector3 operator / (const Vector3& a, double p) {
	return Vector3(a.x / p, a.y / p, a.z / p);
}

double DotProduct(const Vector3& a, const Vector3& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 CrossProduct(const Vector3& a, const Vector3& b) {
  return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

double LengthOfVector3(const Vector3& a) {
	return std::sqrt(DotProduct(a, a));
}

struct PlaneModel {
  Point3 p;
  Vector3 n;
	PlaneModel() = default;

  PlaneModel(Point3 a, Point3 b, Point3 c) {
    p = a;
    n = CrossProduct(b-a, c - a);
		n = n / LengthOfVector3(n);
  }

	void operator=(const PlaneModel& o) {
		p = o.p;
		n = o.n;
	}
};

// Distance from p to the plane (p0, n).
// n needs to have length 1.
double DistanceToPlane(const Point3& p, const Point3& p0, const Vector3& n) {
	// Without fabs, the distance is directed. Positive means it is in the same
	// direction as n.
	return std::fabs(DotProduct(p - p0, n));
}


}
