#include "Core.h"
#include "LocusMathFunctions.h"
namespace bBody {
using badger::Matrix3;
using badger::Matrix4;
using badger::Quaternion;
using badger::real;
using badger::Vector3;
using bMath::BMathFunctions;

class RigidBody {
protected:
  Vector3 forceAccum;
  Vector3 torqueAccum;
  real inverseMass;
  real linearDamping = 0.9;
  real angularDamping = 0.9;
  Vector3 position;
  Vector3 linearVelocity;
  Vector3 angularVelocity;
  Quaternion orientation;
  Matrix4 transformationMatrix;
  Matrix3 inverseInertiaTensorLocal;
  Matrix3 inverseInertiaTensorWorld;

  // rotMatrix . inverseInerialLocal * rotMxtrixInv;
  // think of it like qvq* from quaternion
  static inline void
  _calculateInverseInetiaTensorWorld(Matrix3 &inverseInertiaTensorWorld,
                                     const Matrix4 &transformationMatrix,
                                     const Matrix3 &inverseInertiaTensorLocal);
  // removes force and torque accumulated over a frame
  void clearAccumulators();
  /*
  Should be Called on the body to every frame toset its parameters i.e
  orientaion normalizing
  tx matrix
  Inverse Inertia Tensor in world coordinates
 */
  void calculateDerivedData();

public:
  /*
  Sets a body's inverse inertia tensor
  @param - badger::Matrix3
  */
  void setInverseInertiaTensorLocal(const Matrix3 &inertiaTensor);
  /*
  Apply force on an objects world point that
  may result in rotation
  @param force - in world space
  @param point - in world
  */
  void addForceAtPoint(const Vector3 &force, const Vector3 &point);
  /*
  Apply force on an objects local point that
  may result in rotation
  @param force - in world space
  @param point - in objects localspace
  */
  void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
  // Apply force for linear motion
  void addForce(const Vector3 &force);
  void integrate(real duration);
  /*Check whether the body is immovable
  returns 1 if immovable, 0 if movable
  Note if inverse Mass is less than 0.001f its considered immovable/static
  */
  int8_t hasInfiniteMass() const;
  /*
  Converts a world point to local coordinate on rigidbody
  @param -badger::Vector3 world point point
  */
  Vector3 getPointLocal(const Vector3 &worldpos) const;
  /*
  Converts a rigid body's local point to world coordinate
  @param -badger::Vector3 local point
  */
  Vector3 getPointWorld(const Vector3 &localpos) const;
};
} // namespace locusBody
