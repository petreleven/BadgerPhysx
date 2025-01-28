#include "World.h"
#include "RigidBody.h"


using badger::World;

void World::registerBodyForce(bBody::RigidBody *b, ForceGenerator *g) {
  RigidBodyForceRegister r(b, g);
  bf_register.push_back(r);
}

void World::registerBody(bBody::RigidBody *b) {
  if (b != nullptr)
    b_register.push_back(b);
}

void World::startFrame(){
	for(bBody::RigidBody *b : b_register){
		if(b!=nullptr){
			b->clearAccumulators();
			b->calculateDerivedData();
		}
	}
}

void World::update(real dt){
	for(RigidBodyForceRegister &bodyForce : bf_register){
		bodyForce.update(dt);
	}
	for(bBody::RigidBody *body : b_register){
		if(body == nullptr){
			continue;
		}
		body->integrate(dt);
	}
}