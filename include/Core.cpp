#include "Core.h"
#include <raylib.h>

namespace badger {

// Matrix3
Vector3 Matrix3::operator*(const Vector3 &p) const {
  return Vector3(data[0] * p.x + data[1] * p.y + data[2] * p.z,
                 data[3] * p.x + data[4] * p.y + data[5] * p.z,
                 data[6] * p.x + data[7] * p.y + data[8] * p.z);
}

Matrix3 Matrix3::operator*(const Matrix3 &m) const {
  Matrix3 newM;
  newM.data[0] = (*this).data[0] * m.data[0] + (*this).data[1] * m.data[3] +
                 (*this).data[2] * m.data[6];
  newM.data[1] = (*this).data[0] * m.data[1] + (*this).data[1] * m.data[4] +
                 (*this).data[2] * m.data[7];
  newM.data[2] = (*this).data[0] * m.data[2] + (*this).data[1] * m.data[5] +
                 (*this).data[2] * m.data[8];

  newM.data[3] = (*this).data[3] * m.data[0] + (*this).data[4] * m.data[3] +
                 (*this).data[5] * m.data[6];
  newM.data[4] = (*this).data[3] * m.data[1] + (*this).data[4] * m.data[4] +
                 (*this).data[5] * m.data[7];
  newM.data[5] = (*this).data[3] * m.data[2] + (*this).data[4] * m.data[5] +
                 (*this).data[5] * m.data[8];

  newM.data[6] = (*this).data[6] * m.data[0] + (*this).data[7] * m.data[3] +
                 (*this).data[8] * m.data[6];
  newM.data[7] = (*this).data[6] * m.data[1] + (*this).data[7] * m.data[4] +
                 (*this).data[8] * m.data[7];
  newM.data[8] = (*this).data[6] * m.data[2] + (*this).data[7] * m.data[5] +
                 (*this).data[8] * m.data[8];
  return newM;
}

void Matrix3::scale(real scalar) {
  for (size_t i = 0; i < 9; i++) {
    data[i] *= scalar;
  }
}

Vector3 Matrix3::transform(const Vector3 &p) const { return (*this) * p; }

void Matrix3::setInverse(const Matrix3 &m) {
  real t1 = m.data[0] * m.data[4] * m.data[8];
  real t2 = m.data[3] * m.data[7] * m.data[2];
  real t3 = m.data[6] * m.data[1] * m.data[5];
  real t4 = m.data[0] * m.data[7] * m.data[5];
  real t5 = m.data[6] * m.data[4] * m.data[2];
  real t6 = m.data[3] * m.data[1] * m.data[8];
  real determinant = t1 + t2 + t3 - t4 - t5 - t6;
  if (determinant == 0.0f) {
    return;
  }

  data[0] = (m.data[4] * m.data[8]) - (m.data[5] * m.data[7]);
  data[1] = -((m.data[1] * m.data[8]) - (m.data[2] * m.data[7]));
  data[2] = (m.data[1] * m.data[5]) - (m.data[2] * m.data[4]);
  data[3] = -((m.data[3] * m.data[8]) - (m.data[5] * m.data[6]));
  data[4] = ((m.data[0] * m.data[8]) - (m.data[2] * m.data[6]));
  data[5] = -((m.data[0] * m.data[5]) - (m.data[2] * m.data[3]));
  data[6] = (m.data[3] * m.data[7]) - (m.data[4] * m.data[6]);
  data[7] = -((m.data[0] * m.data[7]) - (m.data[1] * m.data[6]));
  data[8] = (m.data[0] * m.data[4]) - (m.data[1] * m.data[3]);
  this->scale(1 / determinant);
}
Matrix3 Matrix3::inverse() const {
  Matrix3 newM;
  newM.setInverse(*this);
  return newM;
}
void Matrix3::invert() { setInverse(*this); }

void Matrix3::setTranspose(const Matrix3 &m) {
  data[0] = m.data[0];
  data[1] = m.data[3];
  data[2] = m.data[6];
  data[3] = m.data[1];
  data[4] = m.data[4];
  data[5] = m.data[7];
  data[6] = m.data[2];
  data[7] = m.data[5];
  data[8] = m.data[8];
}
Matrix3 Matrix3::transpose() const {
  Matrix3 m;
  m.setTranspose(*this);
  return m;
}

/**
 * Sets this matrix to be the rotation matrix corresponding to
 * the given quaternion.
 */
void Matrix3::setOrientation(const Quaternion &q) {
  data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
  data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
  data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
  data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
  data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
  data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
  data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
  data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
  data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
}
// Matrix4
Matrix4 Matrix4::operator*(const Matrix4 &o) const {
  Matrix4 result;
  result.data[0] =
      o.data[0] * data[0] + o.data[4] * data[1] + o.data[8] * data[2];
  result.data[4] =
      o.data[0] * data[4] + o.data[4] * data[5] + o.data[8] * data[6];
  result.data[8] =
      o.data[0] * data[8] + o.data[4] * data[9] + o.data[8] * data[10];
  result.data[1] =
      o.data[1] * data[0] + o.data[5] * data[1] + o.data[9] * data[2];
  result.data[5] =
      o.data[1] * data[4] + o.data[5] * data[5] + o.data[9] * data[6];
  result.data[9] =
      o.data[1] * data[8] + o.data[5] * data[9] + o.data[9] * data[10];
  result.data[2] =
      o.data[2] * data[0] + o.data[6] * data[1] + o.data[10] * data[2];
  result.data[6] =
      o.data[2] * data[4] + o.data[6] * data[5] + o.data[10] * data[6];
  result.data[10] =
      o.data[2] * data[8] + o.data[6] * data[9] + o.data[10] * data[10];
  result.data[3] = o.data[3] * data[0] + o.data[7] * data[1] +
                   o.data[11] * data[2] + data[3];
  result.data[7] = o.data[3] * data[4] + o.data[7] * data[5] +
                   o.data[11] * data[6] + data[7];
  result.data[11] = o.data[3] * data[8] + o.data[7] * data[9] +
                    o.data[11] * data[10] + data[11];
  return result;
}
Vector3 Matrix4::transform(const Vector3 &p) const { return (*this) * p; }
Vector3 Matrix4::operator*(const Vector3 &p) const {
  return Vector3(data[0] * p.x + data[1] * p.y + data[2] * p.z + data[3],
                 data[4] * p.x + data[5] * p.y + data[6] * p.z + data[7],
                 data[8] * p.x + data[9] * p.y + data[10] * p.z + data[11]);
}
Matrix4 Matrix4::inverse() const {
  Matrix4 result;
  result.setInverse(*this);
  return result;
}
void Matrix4::invert() { setInverse(*this); }

real Matrix4::getDeterminant() const {
  return data[8] * data[5] * data[2] + data[4] * data[9] * data[2] +
         data[8] * data[1] * data[6] - data[0] * data[9] * data[6] -
         data[4] * data[1] * data[10] + data[0] * data[5] * data[10];
}
void Matrix4::setInverse(const Matrix4 &m) {
  // Make sure the determinant is non-zero.
  real det = getDeterminant();
  if (det == 0)
    return;
  det = ((real)1.0) / det;

  data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
  data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
  data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * det;

  data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
  data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
  data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * det;

  data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * det;
  data[6] = (+m.data[4] * m.data[2] - m.data[0] * m.data[6]) * det;
  data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * det;

  data[3] =
      (m.data[9] * m.data[6] * m.data[3] - m.data[5] * m.data[10] * m.data[3] -
       m.data[9] * m.data[2] * m.data[7] + m.data[1] * m.data[10] * m.data[7] +
       m.data[5] * m.data[2] * m.data[11] -
       m.data[1] * m.data[6] * m.data[11]) *
      det;
  data[7] =
      (-m.data[8] * m.data[6] * m.data[3] + m.data[4] * m.data[10] * m.data[3] +
       m.data[8] * m.data[2] * m.data[7] - m.data[0] * m.data[10] * m.data[7] -
       m.data[4] * m.data[2] * m.data[11] +
       m.data[0] * m.data[6] * m.data[11]) *
      det;
  data[11] =
      (m.data[8] * m.data[5] * m.data[3] - m.data[4] * m.data[9] * m.data[3] -
       m.data[8] * m.data[1] * m.data[7] + m.data[0] * m.data[9] * m.data[7] +
       m.data[4] * m.data[1] * m.data[11] -
       m.data[0] * m.data[5] * m.data[11]) *
      det;
}
/**
 * Sets this matrix to be the rotation matrix corresponding to
 * the given quaternion.
 */
void Matrix4::setOrientation(const Quaternion &q, const Vector3 &pos) {
  data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
  data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
  data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
  data[3] = pos.x;

  data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
  data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
  data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
  data[7] = pos.y;

  data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
  data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
  data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
  data[11] = pos.z;
}

//Convert  local point to world coordinates
Vector3 Matrix4::localToWorld(const Vector3 &v,
                              const Matrix4 &transform) const {
  return transform.transform(v);
}

//Convert  world point to local coordinates
Vector3 Matrix4::worldToLocal(const Vector3 &v,
                              const Matrix4 &transform) const {
  /*Matrix4 invM;
  invM.setInverse(transform);
  return invM.transform(v);*/
  // MTranspose *(V-T)
  Vector3 tmp = v;
  tmp.x -= transform.data[3];
  tmp.y -= transform.data[7];
  tmp.z -= transform.data[11];
  return Vector3(transform.data[0] * tmp.x + transform.data[4] * tmp.y +
                     transform.data[8] * tmp.z,
                 transform.data[1] * tmp.x + transform.data[5] * tmp.y +
                     transform.data[9] * tmp.z,
                 transform.data[2] * tmp.x + transform.data[6] * tmp.y +
                     transform.data[10] * tmp.z);
}

Vector3 Matrix4::transformDirection(const Vector3 &v) const {
  return Vector3(data[0] * v.x + data[1] * v.y + data[2] * v.z,
                 data[4] * v.x + data[5] * v.y + data[6] * v.z,
                 data[8] * v.x + data[9] * v.y + data[10] * v.z);
}
Vector3 Matrix4::transformInVerseDirection(const Vector3 &v) const {
  return Vector3(data[0] * v.x + data[4] * v.y + data[8] * v.z,
                 data[1] * v.x + data[5] * v.y + data[9] * v.z,
                 data[2] * v.x + data[6] * v.y + data[10] * v.z);
}

Vector3 Matrix4::localToWorldDirection(const Vector3 &v,
                                       const Matrix4 &transform) const {
  return transform.transformDirection(v);
}
Vector3 Matrix4::worldToLocalDirection(const Vector3 &v,
                                       const Matrix4 &transform) const {
  return transform.transformInVerseDirection(v);
}
} // namespace badger
