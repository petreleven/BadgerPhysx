#include "ForceGenerator.h"

using bMath::BMathFunctions;
using namespace badger;
void GravityForceGenerator::applyForce(RigidBody *rigidBody, real dt) const {
  if (rigidBody->hasInfiniteMass()) {
    return;
  }
  rigidBody->addForce(gravity);
}

void SpringForceGenerator::applyForce(RigidBody *body, const real &dt) const {
  badger::Vector3 suspensionPointWorld = other->getPointWorld(suspensionPoint);
  badger::Vector3 connectionPointWorld = body->getPointWorld(connectionPoint);
  badger::Vector3 diff = (connectionPoint - suspensionPoint);
  real dx = BMathFunctions::Magnitude(diff);
  // f = -kx;
  real forceMag = -springConstant * std::abs(dx - restLength);
  badger::Vector3 forceAndDir = BMathFunctions::Normalize(diff);
  forceAndDir *= forceMag;
  body->addForceAtPoint(forceAndDir, connectionPointWorld);
}
