#include "ForceGenerator.h"


using bMath::BMathFunctions;
using namespace badger;
void GravityForceGenerator::applyForce(RigidBody *rigidBody,const real &dt) const {
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

RotationalForceGeneraor::RotationalForceGeneraor(real scale_, badger::Vector3 relativePos_,
                          badger::Vector3 direction_){
  scale = scale_;
  relativePos = relativePos_;
  direction = direction_;
  direction = bMath::BMathFunctions::Normalize(direction);
}


void RotationalForceGeneraor::applyForce(RigidBody* rigidBody, const real& dt) const {
    Vector3 newDir = rigidBody->getTransform().transformDirection(direction);
    newDir = bMath::BMathFunctions::Normalize(newDir);
    // Reduce force as angular velocity increases
    Vector3 currentAngVel = rigidBody->getAngularVel();
    real dampingFactor = 1.0f  / (1+bMath::BMathFunctions::Magnitude(currentAngVel));
    Vector3 scaledForce = newDir * (scale * dampingFactor);
    
    rigidBody->addForceAtBodyPoint(scaledForce, relativePos);
    rigidBody->addForceAtBodyPoint(scaledForce * -1.0f, relativePos * -1.0f);
}