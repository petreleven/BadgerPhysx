#include "RigidBody.h"
#include "LocusMathFunctions.h"



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
void RigidBody::calculateDerivedData() {
  orientation.normalize();
  transformationMatrix.setOrientation(orientation, position);
  _calculateInverseInetiaTensorWorld(inverseInertiaTensorWorld,
                                     transformationMatrix,
                                     inverseInertiaTensorLocal);
}

void RigidBody::setInverseInertiaTensorLocal(const Matrix3 &inertiaTensor) {
  inverseInertiaTensorLocal.setInverse(inertiaTensor);
}

inline void RigidBody::_calculateInverseInetiaTensorWorld(
    Matrix3 &inverseInertiaTensorWorld, const Matrix4 &transformationMatrix,
    const Matrix3 &inverseInertiaTensorLocal) {

  inverseInertiaTensorWorld.data[0] =
      transformationMatrix.data[0] * inverseInertiaTensorLocal.data[0] +
      transformationMatrix.data[1] * inverseInertiaTensorLocal.data[3] +
      transformationMatrix.data[2] * inverseInertiaTensorLocal.data[6];
  inverseInertiaTensorWorld.data[1] =
      transformationMatrix.data[0] * inverseInertiaTensorLocal.data[1] +
      transformationMatrix.data[1] * inverseInertiaTensorLocal.data[4] +
      transformationMatrix.data[2] * inverseInertiaTensorLocal.data[7];
  inverseInertiaTensorWorld.data[2] =
      transformationMatrix.data[0] * inverseInertiaTensorLocal.data[2] +
      transformationMatrix.data[1] * inverseInertiaTensorLocal.data[5] +
      transformationMatrix.data[2] * inverseInertiaTensorLocal.data[8];

  inverseInertiaTensorWorld.data[3] =
      transformationMatrix.data[4] * inverseInertiaTensorLocal.data[0] +
      transformationMatrix.data[5] * inverseInertiaTensorLocal.data[3] +
      transformationMatrix.data[6] * inverseInertiaTensorLocal.data[6];
  inverseInertiaTensorWorld.data[4] =
      transformationMatrix.data[4] * inverseInertiaTensorLocal.data[1] +
      transformationMatrix.data[5] * inverseInertiaTensorLocal.data[4] +
      transformationMatrix.data[6] * inverseInertiaTensorLocal.data[7];
  inverseInertiaTensorWorld.data[5] =
      transformationMatrix.data[4] * inverseInertiaTensorLocal.data[2] +
      transformationMatrix.data[5] * inverseInertiaTensorLocal.data[5] +
      transformationMatrix.data[6] * inverseInertiaTensorLocal.data[8];

  inverseInertiaTensorWorld.data[6] =
      transformationMatrix.data[8] * inverseInertiaTensorLocal.data[0] +
      transformationMatrix.data[9] * inverseInertiaTensorLocal.data[3] +
      transformationMatrix.data[10] * inverseInertiaTensorLocal.data[6];
  inverseInertiaTensorWorld.data[7] =
      transformationMatrix.data[8] * inverseInertiaTensorLocal.data[1] +
      transformationMatrix.data[9] * inverseInertiaTensorLocal.data[4] +
      transformationMatrix.data[10] * inverseInertiaTensorLocal.data[7];
  inverseInertiaTensorWorld.data[8] =
      transformationMatrix.data[8] * inverseInertiaTensorLocal.data[2] +
      transformationMatrix.data[9] * inverseInertiaTensorLocal.data[5] +
      transformationMatrix.data[10] * inverseInertiaTensorLocal.data[8];
  // by R*
  real data[9];
  data[0] = inverseInertiaTensorWorld.data[0] * transformationMatrix.data[0] +
            inverseInertiaTensorWorld.data[1] * transformationMatrix.data[1] +
            inverseInertiaTensorWorld.data[2] * transformationMatrix.data[2];
  data[1] = inverseInertiaTensorWorld.data[0] * transformationMatrix.data[4] +
            inverseInertiaTensorWorld.data[1] * transformationMatrix.data[5] +
            inverseInertiaTensorWorld.data[2] * transformationMatrix.data[6];
  data[2] = inverseInertiaTensorWorld.data[0] * transformationMatrix.data[8] +
            inverseInertiaTensorWorld.data[1] * transformationMatrix.data[9] +
            inverseInertiaTensorWorld.data[2] * transformationMatrix.data[10];

  data[3] = inverseInertiaTensorWorld.data[3] * transformationMatrix.data[0] +
            inverseInertiaTensorWorld.data[4] * transformationMatrix.data[1] +
            inverseInertiaTensorWorld.data[5] * transformationMatrix.data[2];
  data[4] = inverseInertiaTensorWorld.data[3] * transformationMatrix.data[4] +
            inverseInertiaTensorWorld.data[4] * transformationMatrix.data[5] +
            inverseInertiaTensorWorld.data[5] * transformationMatrix.data[6];
  data[5] = inverseInertiaTensorWorld.data[3] * transformationMatrix.data[8] +
            inverseInertiaTensorWorld.data[4] * transformationMatrix.data[9] +
            inverseInertiaTensorWorld.data[5] * transformationMatrix.data[10];

  data[6] = inverseInertiaTensorWorld.data[6] * transformationMatrix.data[0] +
            inverseInertiaTensorWorld.data[7] * transformationMatrix.data[1] +
            inverseInertiaTensorWorld.data[8] * transformationMatrix.data[2];
  data[7] = inverseInertiaTensorWorld.data[6] * transformationMatrix.data[4] +
            inverseInertiaTensorWorld.data[7] * transformationMatrix.data[5] +
            inverseInertiaTensorWorld.data[8] * transformationMatrix.data[6];
  data[8] = inverseInertiaTensorWorld.data[6] * transformationMatrix.data[8] +
            inverseInertiaTensorWorld.data[7] * transformationMatrix.data[9] +
            inverseInertiaTensorWorld.data[8] * transformationMatrix.data[10];

  inverseInertiaTensorWorld.data[0] = data[0];
  inverseInertiaTensorWorld.data[1] = data[1];
  inverseInertiaTensorWorld.data[2] = data[2];
  inverseInertiaTensorWorld.data[3] = data[3];
  inverseInertiaTensorWorld.data[4] = data[4];
  inverseInertiaTensorWorld.data[5] = data[5];
  inverseInertiaTensorWorld.data[6] = data[6];
  inverseInertiaTensorWorld.data[7] = data[7];
  inverseInertiaTensorWorld.data[8] = data[8];
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
