/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/odom.h"

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv) {

  mallopt(M_ARENA_MAX, 1);
  
  ros::init(argc, argv, "dlio_odom_node" , ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  signal(SIGINT, mySigintHandler);

  dlio::OdomNode node(nh);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;

}
