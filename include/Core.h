#pragma once

#include <algorithm>
#include <bits/types/struct_tm.h>
#include <cstddef>
#include <math.h>
#include <raylib.h>
namespace badger {
typedef float real;
real EPSILON = 1e-6f;
class Vector3 {
public:
  real x;
  real y;
  real z;

private:
  real pad;
  real real_sqrt(real squareMag) { return sqrtf(squareMag); }

public:
  Vector3() : x(0), y(0), z(0) {}
  Vector3(const real x, const real y, const real z) : x(x), y(y), z(z) {}

  // ADD TO THIS
  void operator+=(const Vector3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  Vector3 operator+(const Vector3 &v)  const{
    return Vector3(x + v.x,
    y + v.y,
    z + v.z);
  }
 
  Vector3 operator+=(badger::real &sc) {
    x += sc;
    y += sc;
    z += sc;
    return Vector3(x, y, z);
  }

  Vector3 &operator-=(const Vector3 &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
  }

  Vector3 operator-(const Vector3 &v) const {
    return Vector3(
    x - v.x,
    y - v.y,
    z - v.z);
  }
 

  Vector3 &operator=(const Vector3 &v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }

  Vector3 &operator*=(const real &scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }
  Vector3 operator*(const real &scalar) {
    return Vector3(x * scalar, y * scalar, z * scalar);
  }
};

class Quaternion {
public:
  real r;
  real i;
  real j;
  real k;
  /**
   *Default constructor creates a quarernion with no rotation
   **/
  Quaternion() : r(1), i(0), j(0), k(0){};
  /**
   *Creates  quaternion without normalising
   *see @normalize
   **/
  Quaternion(real r, real i, real j, real k) : r(r), i(i), j(j), k(k) {}
  /*
  *Normalizes the quaternion in place
  */
  void normalize() {
    real mag = sqrtf(r * r + i * i + j * j + k * k);
    if (mag < EPSILON) {
      r = 1;
      return;
    }
    real Invmag = 1 / mag;
    r = r * Invmag;
    i = i * Invmag;
    j = j * Invmag;
    k = k * Invmag;
  }
  Quaternion operator*(const Quaternion &multiplier) const {
    const Quaternion &tmp = *this;
    Quaternion result;
    result.r = tmp.r * multiplier.r - tmp.i * multiplier.i -
               tmp.j * multiplier.j - tmp.k * multiplier.k;
    result.i = tmp.r * multiplier.i + tmp.i * multiplier.r +
               tmp.j * multiplier.k - tmp.k * multiplier.j;
    result.j = tmp.r * multiplier.j - tmp.i * multiplier.k +
               tmp.j * multiplier.r + tmp.k * multiplier.i;
    result.k = tmp.r * multiplier.k + tmp.i * multiplier.j -
               tmp.j * multiplier.i + tmp.k * multiplier.r;
    return result;
  }
  Quaternion &operator*=(const Quaternion &multiplier) {
    Quaternion tmp = *this;
    r = tmp.r * multiplier.r - tmp.i * multiplier.i - tmp.j * multiplier.j -
        tmp.k * multiplier.k;
    i = tmp.r * multiplier.i + tmp.i * multiplier.r + tmp.j * multiplier.k -
        tmp.k * multiplier.j;
    j = tmp.r * multiplier.j - tmp.i * multiplier.k + tmp.j * multiplier.r +
        tmp.k * multiplier.i;
    k = tmp.r * multiplier.k + tmp.i * multiplier.j - tmp.j * multiplier.i +
        tmp.k * multiplier.r;
    return (*this);
  }
  /*
   * Rotate quaternion using a vector
   * @param vector - Rotation Vector
   */
  void rotateByVector(const Vector3 &v) {
    Quaternion tmp(0, v.x, v.y, v.z);
    *this *= tmp;
  }
  /**
  Used to rotate  q quaternion over time
  @param vector -The vector to add
  @param scale -The amount of the vector to add
  q = q + wq * 0.5 * dt
  **/
  void addScaledVector(const Vector3 &v, real scale) {
    Quaternion qtemp(0, v.x * scale, v.y * scale, v.z * scale);
    qtemp *= (*this);
    r += qtemp.r * 0.5;
    i += qtemp.i * 0.5;
    j += qtemp.j * 0.5;
    k += qtemp.k * 0.5;
  }
};
class Matrix3 {
public:
  real data[9];

public:
  Vector3 operator*(const Vector3 &p) const;
  Matrix3 operator*(const Matrix3 &m) const;
  void scale(real scalar);
  Vector3 transform(const Vector3 &p) const;
  void setInverse(const Matrix3 &m);
  Matrix3 inverse() const;
  void invert();
  void setTranspose(const Matrix3 &m);
  Matrix3 transpose() const;
  void setOrientation(const Quaternion &q);
};

class Matrix4 {
public:
  real data[12];
  Vector3 operator*(const Vector3 &p) const;
  Vector3 transform(const Vector3 &p) const;
  Matrix4 operator*(const Matrix4 &o) const;
  real getDeterminant() const;
  void setInverse(const Matrix4 &m);
  Matrix4 inverse() const;
  void invert();
  void setOrientation(const Quaternion &q, const Vector3 &pos);
  Vector3 localToWorld(const Vector3 &v, const Matrix4 &transform) const;
  Vector3 worldToLocal(const Vector3 &v, const Matrix4 &transform) const;
  Vector3 transformDirection(const Vector3 &v) const;
  Vector3 transformInVerseDirection(const Vector3 &v) const;
  Vector3 localToWorldDirection(const Vector3 &v,
                                const Matrix4 &transform) const;
  Vector3 worldToLocalDirection(const Vector3 &v,
                                const Matrix4 &transform) const;
};

struct Point {
  real x;
  real y;
  real z;
  void operator+=(const Vector3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
  }
  Point operator+(badger::real &sc) {
    x += sc;
    y += sc;
    z += sc;
    return Point{x, y, z};
  }

  // SUBSTRACT FROM THIS
  void operator-=(const Point &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
  }
  // SUUBSTRACT AND RETURN NEW Vec3
  Point operator-(const Point v) const {
    return Point{x - v.x, y - v.y, z - v.z};
  }

  Point &operator=(const Point &v) {
    x = v.x;
    y = v.y;
    z = v.z;
    return *this;
  }
};

struct Plane {
  badger::Vector3 n; // plane normal
  badger::real d;    // distance from origin dot(normal, point_on_plane)
};

}; // namespace badger
