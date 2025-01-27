#include "Core.h"
#include "LocusMathFunctions.h"
#include "RigidBody.h"

using badger::real;
using bBody::RigidBody;

class ForceGenerator {
public:
  virtual void applyForce(RigidBody *rigidBody, real dt) const = 0;
};

class GravityForceGenerator : public ForceGenerator {
  badger::Vector3 gravity;

public:
  GravityForceGenerator(const badger::Vector3 &g) : gravity(g){};
  virtual void applyForce(RigidBody *rigidBody, real dt) const;
};

class SpringForceGenerator : public ForceGenerator {
  real springConstant;
  real restLength;
  badger::Vector3 suspensionPoint;
  badger::Vector3 connectionPoint;
  RigidBody *other;

public:
  /*
  Creates a spring force
  @param - badger::real - Spring Constant
  @param - badger::real - restLength
  @param - badger::Vector3 - Suspension local space of springBody;
  @param - badger::Vector3 - Connection Point in local space of suspended body
  @param - bBody::RigidBody *  - Rigidbody where spring is suspended from
  */
  SpringForceGenerator(const real &k, const real &length,
                       const badger::Vector3 p, badger::Vector3 c,
                       RigidBody *other)
      : springConstant(k), restLength(length), suspensionPoint(p),
        connectionPoint(c), other(other){};
  /*
  Apply spring force to a rigid body
  @param - bBody::RigidBody *  - RigidBody hanging from spring
  @param - badger::Real - timestep
  */
  virtual void applyForce(RigidBody *body, const real &dt) const;
};
