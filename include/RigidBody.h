#pragma once
#include "Core.h"
#include "LocusMathFunctions.h"
#include <algorithm>

namespace bBody {
using badger::Matrix3;
using badger::Matrix4;
using badger::Quaternion;
using badger::real;
using badger::Vector3;

class RigidBody {
protected:
  Vector3 forceAccum;
  Vector3 torqueAccum;
  real inverseMass = 1;
  real linearDamping = 0.99;
  real angularDamping = 0.90;
  Vector3 position;
  Vector3 linearVelocity;
  Vector3 angularVelocity;
  Quaternion orientation;
  Matrix4 transformationMatrix;
  Matrix3 inverseInertiaTensorLocal;
  Matrix3 inverseInertiaTensorWorld;
  real maxAngularSpeed = 4 * 3.14159265358979323846;
  // rotMatrix . inverseInerialLocal * rotMxtrixInv;
  // think of it like qvq* from quaternion
  static inline void
  _calculateInverseInetiaTensorWorld(Matrix3 &inverseInertiaTensorWorld,
                                     const Matrix4 &transformationMatrix,
                                     const Matrix3 &inverseInertiaTensorLocal);
  // Add method to clamp angular velocity
  void clampAngularVelocity() {
    real currentSpeed = bMath::BMathFunctions::Magnitude(angularVelocity);
    if (currentSpeed > maxAngularSpeed) {
      angularVelocity *= (maxAngularSpeed / currentSpeed);
    }
  }

public:
  /**
   * Sets the position of the rigid body in 3D space.
   * @param v The new position vector to set
   */
  void setPosition(badger::Vector3 v) { position = v; }

  /**
   * Sets the mass of the rigid body and calculates its inverse mass.
   * The inverse mass is stored internally for efficient physics calculations.
   * @param mass The mass value to set (passed by reference)
   */
  void setMass(badger::real &mass) { inverseMass = 1 / mass; }

  /**
   * Sets the orientation of the rigid body using a quaternion.
   * @param q The quaternion representing the new orientation
   */
  void setsOrientation(badger::Quaternion q) { orientation = q; }

  /**
   * Sets the dimensions of the rigid body and calculates its inverse inertia
   * tensor. Uses the standard formulas for a rectangular cuboid's moments of
   * inertia. The inverse inertia tensor is used for rotational physics
   * calculations.
   *
   * @param height The height of the rigid body
   * @param width The width of the rigid body
   * @param depth The depth of the rigid body
   *
   * Formula used for moments of inertia:
   * Ix = (1/12) * mass * (height^2 + depth^2)
   * Iy = (1/12) * mass * (width^2 + depth^2)
   * Iz = (1/12) * mass * (width^2 + height^2)
   */
  void SetDimensions(badger::real height, badger::real width,
                     badger::real depth) {
    height = std::max(height, 0.001f);
    width = std::max(width, 0.001f);
    depth = std::max(depth, 0.001f);
    real mass = 1 / inverseMass;
    mass = std::max(mass, 0.001f);
    real ix = (1.0f / 12.0f) * mass * (height * height + depth * depth);
    real iy = (1.0f / 12.0f) * mass * (width * width + depth * depth);
    real iz = (1.0f / 12.0f) * mass * (width * width + height * height);
    // Ensure minimum inertia values
    ix = std::max(ix, 0.0001f);
    iy = std::max(iy, 0.0001f);
    iz = std::max(iz, 0.0001f);
    badger::Matrix3 iT;
    iT.data[0] = ix;
    iT.data[4] = iy;
    iT.data[8] = iz;
    setInverseInertiaTensorLocal(iT);
  }
  // removes force and torque accumulated over a frame
  void clearAccumulators();
  /*
   *Should be Called on the body to every frame to
   *set its parameters i.e
   *orientaion normalizing
   *recalculates tx matrix
   *precalculates Inverse Inertia Tensor in world coordinates
   */
  void calculateDerivedData();
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
  *Apply force on an objects local point that
  *may result in rotation
  *@param force - in world space
  (@param point - in objects localspace
  */
  void addForceAtBodyPoint(const Vector3 &force, const Vector3 &point);
  // Apply force for linear motion
  void addForce(const Vector3 &force);
  void integrate(real duration);
  /*Check whether the body is immovable
   *returns 1 if immovable, 0 if movable
   *Note if inverse Mass is less than 0.001f its considered immovable/static
   */
  int8_t hasInfiniteMass() const;
  /*
   *Converts a world point to local coordinate on rigidbody
   *@param -badger::Vector3 world point point
   */
  Vector3 getPointLocal(const Vector3 &worldpos) const;
  /*
   *Converts a rigid body's local point to world coordinate
   *@param -badger::Vector3 local point
   */
  Vector3 getPointWorld(const Vector3 &localpos) const;
  /*
   *Get the tx matrix
   *note this is a 3 by 4 matrix
   *@retirn badger::Matrix4
   */
  Matrix4 getTransform() const { return transformationMatrix; }
  /*
   *Get the Position
   *@return badger badger::Vector3
   */
  Vector3 getPosition() const { return position; }
  /*
   *Get the angular Velocity
   *@return badger badger::Vector3
   */
  Vector3 getAngularVel() const { return angularVelocity; }
};

} // namespace bBody
