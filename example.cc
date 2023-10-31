#include <stdlib.h>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"

int main() {
  raisim::World::setActivationKey("/home/youngho_ws/.raisim");
  raisim::World world;
  int gcDim_, gvDim_, nJoints_;

  auto anymal = world.addArticulatedSystem("/home/jakob/youngho_ws/raisimLib/rsc/gripper/gripper.urdf");
  auto ball = world.addSphere(0.1, 1);

  ball->setPosition(Eigen::Vector3d(0,-0.5,0));
  auto ground = world.addGround();
  anymal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  world.setTimeStep(0.002);
  gcDim_ = anymal->getGeneralizedCoordinateDim();
  gvDim_ = anymal->getDOF(); //gvDim = 8, gcDim=9
  std::cout << gvDim_;
  Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
  jointPgain.setZero(); jointPgain.tail(4).setConstant(10.0);
  jointDgain.setZero(); jointDgain.tail(4).setConstant(100);
  anymal->setPdGains(jointPgain, jointDgain);
  
  Eigen::VectorXd pTarget(9), vTarget(8);
  Eigen::Vector2d vTargetcon(2);
  vTargetcon << 1, -1;  
  pTarget.setZero(); vTarget.setZero();
  
  vTarget.tail(2)<<vTargetcon;
  anymal->setPdTarget(pTarget, vTarget);

 
  //anymal->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));
  for(int i=0; i<4;i++){
  world.integrate();
  }
  Eigen::VectorXd rootvTarget(3);
  rootvTarget<<0,0,5;
  vTarget.head(3)<<rootvTarget;
  anymal->setPdTarget(pTarget,vTarget);
  world.integrate();


  /// launch raisim server for visualization. Can be visualized on raisimUnity
  raisim::RaisimServer server(&world);
  server.launchServer();

  while (1) {
    raisim::MSLEEP(2);
    server.integrateWorldThreadSafe();
  }

  server.killServer();
}
