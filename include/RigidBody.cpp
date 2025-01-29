#include "RigidBody.h"

namespace bBody {
using badger::real;
using badger::Vector3;

void RigidBody::integrate(real duration) {
  // linear_acceleration=f * invMass
  Vector3 linearAcc = forceAccum * inverseMass;

  // v = v + at
  linearVelocity = linearVelocity * linearDamping + linearAcc * duration;
  // p = p + vt
  position += linearVelocity * duration;

  // angular_acc = InvInertia . Torque;
  Vector3 angularAcc = inverseInertiaTensorWorld * torqueAccum;

  angularVelocity = angularVelocity * angularDamping + angularAcc * duration;

  clampAngularVelocity();

  orientation.addScaledVector(angularVelocity, duration);

  clearAccumulators();
  calculateDerivedData();
}

void RigidBody::clearAccumulators() {
  forceAccum.x = 0;
  forceAccum.y = 0;
  forceAccum.z = 0;
  torqueAccum.x = 0;
  torqueAccum.y = 0;
  torqueAccum.z = 0;
}

void RigidBody::setInverseInertiaTensorLocal(const Matrix3 &inertiaTensor) {
  inverseInertiaTensorLocal.setInverse(inertiaTensor);
}

static inline void _calculateTransformMatrix(Matrix4 &transformMatrix,
                                             const Vector3 &position,
                                             const Quaternion &orientation) {
  transformMatrix.data[0] =
      1 - 2 * orientation.j * orientation.j - 2 * orientation.k * orientation.k;
  transformMatrix.data[1] =
      2 * orientation.i * orientation.j - 2 * orientation.r * orientation.k;
  transformMatrix.data[2] =
      2 * orientation.i * orientation.k + 2 * orientation.r * orientation.j;
  transformMatrix.data[3] = position.x;

  transformMatrix.data[4] =
      2 * orientation.i * orientation.j + 2 * orientation.r * orientation.k;
  transformMatrix.data[5] =
      1 - 2 * orientation.i * orientation.i - 2 * orientation.k * orientation.k;
  transformMatrix.data[6] =
      2 * orientation.j * orientation.k - 2 * orientation.r * orientation.i;
  transformMatrix.data[7] = position.y;

  transformMatrix.data[8] =
      2 * orientation.i * orientation.k - 2 * orientation.r * orientation.j;
  transformMatrix.data[9] =
      2 * orientation.j * orientation.k + 2 * orientation.r * orientation.i;
  transformMatrix.data[10] =
      1 - 2 * orientation.i * orientation.i - 2 * orientation.j * orientation.j;
  transformMatrix.data[11] = position.z;
}
//Does R T R*
static inline void _transformInertiaTensor(Matrix3 &iitWorld,
                                           const Matrix3 &iitBody,
                                           const Matrix4 &rotmat) {
  real t4 = rotmat.data[0] * iitBody.data[0] +
            rotmat.data[1] * iitBody.data[3] + rotmat.data[2] * iitBody.data[6];
  real t9 = rotmat.data[0] * iitBody.data[1] +
            rotmat.data[1] * iitBody.data[4] + rotmat.data[2] * iitBody.data[7];
  real t14 = rotmat.data[0] * iitBody.data[2] +
             rotmat.data[1] * iitBody.data[5] +
             rotmat.data[2] * iitBody.data[8];
  real t28 = rotmat.data[4] * iitBody.data[0] +
             rotmat.data[5] * iitBody.data[3] +
             rotmat.data[6] * iitBody.data[6];
  real t33 = rotmat.data[4] * iitBody.data[1] +
             rotmat.data[5] * iitBody.data[4] +
             rotmat.data[6] * iitBody.data[7];
  real t38 = rotmat.data[4] * iitBody.data[2] +
             rotmat.data[5] * iitBody.data[5] +
             rotmat.data[6] * iitBody.data[8];
  real t52 = rotmat.data[8] * iitBody.data[0] +
             rotmat.data[9] * iitBody.data[3] +
             rotmat.data[10] * iitBody.data[6];
  real t57 = rotmat.data[8] * iitBody.data[1] +
             rotmat.data[9] * iitBody.data[4] +
             rotmat.data[10] * iitBody.data[7];
  real t62 = rotmat.data[8] * iitBody.data[2] +
             rotmat.data[9] * iitBody.data[5] +
             rotmat.data[10] * iitBody.data[8];

  iitWorld.data[0] =
      t4 * rotmat.data[0] + t9 * rotmat.data[1] + t14 * rotmat.data[2];
  iitWorld.data[1] =
      t4 * rotmat.data[4] + t9 * rotmat.data[5] + t14 * rotmat.data[6];
  iitWorld.data[2] =
      t4 * rotmat.data[8] + t9 * rotmat.data[9] + t14 * rotmat.data[10];
  iitWorld.data[3] =
      t28 * rotmat.data[0] + t33 * rotmat.data[1] + t38 * rotmat.data[2];
  iitWorld.data[4] =
      t28 * rotmat.data[4] + t33 * rotmat.data[5] + t38 * rotmat.data[6];
  iitWorld.data[5] =
      t28 * rotmat.data[8] + t33 * rotmat.data[9] + t38 * rotmat.data[10];
  iitWorld.data[6] =
      t52 * rotmat.data[0] + t57 * rotmat.data[1] + t62 * rotmat.data[2];
  iitWorld.data[7] =
      t52 * rotmat.data[4] + t57 * rotmat.data[5] + t62 * rotmat.data[6];
  iitWorld.data[8] =
      t52 * rotmat.data[8] + t57 * rotmat.data[9] + t62 * rotmat.data[10];
}

void RigidBody::calculateDerivedData() {
  orientation.normalize();

  // Calculate the transform matrix for the body.
  _calculateTransformMatrix(transformationMatrix, position, orientation);

  // Calculate the inertiaTensor in world space.
  _transformInertiaTensor(inverseInertiaTensorWorld,
                          inverseInertiaTensorLocal, transformationMatrix);
}

void RigidBody::addForce(const Vector3 &force) { forceAccum += force; }

void RigidBody::addForceAtBodyPoint(const Vector3 &force,
                                    const Vector3 &point) {
  Vector3 pointWorldSpace = getPointWorld(point);
  addForceAtPoint(force, pointWorldSpace);
}

void RigidBody::addForceAtPoint(const Vector3 &force, const Vector3 &point) {
  // t = pt * f
  Vector3 pt = point;
  pt -= position;
  torqueAccum += bMath::BMathFunctions::CrossProduct(pt, force);
  forceAccum += force;
}

int8_t RigidBody::hasInfiniteMass() const {
  if (inverseMass <= 0.001f) {
    return 1;
  }
  return 0;
}

Vector3 RigidBody::getPointWorld(const Vector3 &localpos) const {
  return transformationMatrix.localToWorld(localpos, transformationMatrix);
}

Vector3 RigidBody::getPointLocal(const Vector3 &worldpos) const {
  return transformationMatrix.worldToLocal(worldpos, transformationMatrix);
}

} // namespace bBody
