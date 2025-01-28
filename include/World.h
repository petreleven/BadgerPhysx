#pragma once
#include "Core.h"
#include "ForceGenerator.h"
#include "RigidBody.h"
#include <vector>
namespace badger {
class World {
private:
  struct RigidBodyForceRegister {
  	bBody::RigidBody *b;
  	ForceGenerator *g;
    RigidBodyForceRegister(bBody::RigidBody *b_, ForceGenerator *g_){
    	b = b_;
    	g = g_;
    }
    void update(real dt){
    	if(g!=nullptr && b!=nullptr)
    	g->applyForce(b, dt);
    };
  };
  using RegisterBF = std::vector<RigidBodyForceRegister> ;
  using RegisterB = std::vector<RigidBody *> ;
  RegisterBF bf_register; 
  RegisterB b_register;

public:
	/*
	*Adds rigid body and force affecting it to the engine
	*@param - bBody::RigidBody *
	*@param - ForceGenrator *
	*@return - void
	*/
	void registerBodyForce(bBody::RigidBody *b, ForceGenerator *g);
	/*
	*Adds rigid body to be considered by the engine 
	*@param - bBody::RigidBody *
	*@return - void
	*/
	void registerBody(bBody::RigidBody *b);
	/*
	*This should be called once in update loop
	*tells the engine to recalculate rigid body attributes
	*such as resetting force, recalculating tx matrix etc
	*@return - void
	*/
	void startFrame();
	/*
	*This should be called once in update loop ideally after startframe
	*tells the engine to update rigid body attributes
	*such as resetting force, recalculating tx matrix etc
	*@return - void
	*/
	void update(real dt);
};
} // namespace badger