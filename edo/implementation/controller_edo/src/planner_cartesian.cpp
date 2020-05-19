#include "controller_2link_planar/planner_cartesian.h"

#include <trajectory_msgs/JointTrajectoryPoint.h>

void planner_cartesian::Prepare(void)
{
 RunPeriod = RUN_PERIOD_DEFAULT;

 /* Retrieve parameters from ROS parameter server */
 std::string FullParamName;

 // run_period
 FullParamName = ros::this_node::getName()+"/run_period";

 if (false == Handle.getParam(FullParamName, RunPeriod))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // start_delay
 FullParamName = ros::this_node::getName()+"/start_delay";

 if (false == Handle.getParam(FullParamName, start_delay))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // max_acc
 FullParamName = ros::this_node::getName()+"/max_acc";

 if (false == Handle.getParam(FullParamName, max_acc))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // max_vel
 FullParamName = ros::this_node::getName()+"/max_vel";

 if (false == Handle.getParam(FullParamName, max_vel))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // pi
 FullParamName = ros::this_node::getName()+"/pi";

 if (false == Handle.getParam(FullParamName, pi))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());
 else
  num_coordinate = (unsigned int)pi.size();

 // pf
 FullParamName = ros::this_node::getName()+"/pf";

 if (false == Handle.getParam(FullParamName, pf))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // elbow_high
 FullParamName = ros::this_node::getName()+"/elbow_high";

 if (false == Handle.getParam(FullParamName, elbow_high))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 //shoulder_right
 FullParamName = ros::this_node::getName()+"/shoulder_right";

 if (false == Handle.getParam(FullParamName, shoulder_right))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 // link_length
 FullParamName = ros::this_node::getName()+"/link_length";

 if (false == Handle.getParam(FullParamName, link_length))
  ROS_ERROR("Node %s: unable to retrieve parameter %s.", ros::this_node::getName().c_str(), FullParamName.c_str());

 /* ROS topics */
 jointTrajectory_publisher = Handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/joint_trajectory", 1);
 cartesianTrajectory_publisher = Handle.advertise<trajectory_msgs::JointTrajectoryPoint>("/cartesian_trajectory", 1);

 /* Initialize node state */
 time_from_start = ros::Duration(0.0);

 cartesian_planner = NULL;
 cartesian_planner = new cartesian_planning();
      
 cartesian_planner->init_line(pi.data(), pf.data(), max_vel, max_acc, num_coordinate);
}

void planner_cartesian::RunPeriodically(float Period)
{
 ros::Rate LoopRate(1.0/Period);

 ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);

 while (ros::ok())
 {
  PeriodicTask();

  ros::spinOnce();

  LoopRate.sleep();
 }
}

void planner_cartesian::Shutdown(void)
{
 if (cartesian_planner)
 {
   delete cartesian_planner;
 }

 ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void planner_cartesian::PeriodicTask(void)
{
  /* Time update */
  if (ros::Time::now().toSec() <= start_delay) {
    time_from_start = ros::Duration(0.0);
  } else {
    time_from_start += ros::Duration(RunPeriod);
  }

  /* Compute a new cartesian trajectory point */
  double p[num_coordinate], pv[num_coordinate], pa[num_coordinate];  
  cartesian_planner->plan(time_from_start.toSec());
  cartesian_planner->getCartesianPosition(p);
  cartesian_planner->getCartesianVelocity(pv);
  cartesian_planner->getCartesianAcceleration(pa);

  /* Convert a cartesian into a joint trajectory */
  double q[num_coordinate], qv[num_coordinate];
  inverse_kinematics(p, q, elbow_high, shoulder_right);
  inverse_diffkinematics(pv, q, qv);

  /* Publish a joint trajectory point */
  trajectory_msgs::JointTrajectoryPoint jointTrajectory;
  jointTrajectory.time_from_start = time_from_start;

  jointTrajectory.positions.clear();
  jointTrajectory.velocities.clear();
  jointTrajectory.accelerations.clear();
  for (int i=0; i<num_coordinate; i++)
  {
    jointTrajectory.positions.push_back(q[i]);
    jointTrajectory.velocities.push_back(qv[i]);
  }

  jointTrajectory_publisher.publish(jointTrajectory);

  /* Publish a cartesian trajectory point */
  trajectory_msgs::JointTrajectoryPoint cartesianTrajectory;
  cartesianTrajectory.time_from_start = time_from_start;

  cartesianTrajectory.positions.clear();
  cartesianTrajectory.velocities.clear();
  cartesianTrajectory.accelerations.clear();
  for (int i=0; i<num_coordinate; i++)
  {
    cartesianTrajectory.positions.push_back(p[i]);
    cartesianTrajectory.velocities.push_back(pv[i]);
    cartesianTrajectory.accelerations.push_back(pa[i]);
  }

  cartesianTrajectory_publisher.publish(cartesianTrajectory);
}

void planner_cartesian::inverse_kinematics(double p[], double q[], bool elbow_high, bool shoulder_right){
    ROS_INFO("L1: %f", link_length.at(0));
    ROS_INFO("L2: %f", link_length.at(1));
    ROS_INFO("L3: %f", link_length.at(2));

    double c3 = (pow(p[0], 2.0) + pow(p[1], 2.0) + pow(p[2], 2.0) - pow(link_length.at(1), 2.0) - pow(link_length.at(2), 2.0)) /
                ( 2.0 * link_length.at(1) * link_length.at(2) );
    
    double s3;
    if (shoulder_right){
      s3 = pow((1.0 - pow(c3, 2.0)), 0.5);
    }
    else{
      s3 = -pow((1.0 - pow(c3, 2.0)), 0.5);
    }

    int sgn;
    if (elbow_high){
      sgn = 1;
    } else {
      sgn = -1;
    }

    ROS_INFO("sign: %d", sgn);
    ROS_INFO("elbow_high: %d", elbow_high);
    ROS_INFO("shoulder_right: %d", shoulder_right);

    double c2 = ( sgn * ( pow(pow(p[0], 2.0) + pow(p[1], 2.0), 0.5) ) * (link_length.at(1) + link_length.at(2) * c3) + p[2] * link_length.at(2) * s3 ) /
                ( pow(link_length.at(1), 2.0) + pow(link_length.at(2), 2.0) + 2.0 * link_length.at(1) * link_length.at(2) * c3 );

    double s2 = ( p[2] * (link_length.at(1) + link_length.at(2) * c3) + (-1*sgn) * ( pow(pow(p[0], 2.0) + pow(p[1], 2.0), 0.5) ) * link_length.at(2) * s3 ) /
                ( pow(link_length.at(1), 2.0) + pow(link_length.at(2), 2.0) + 2.0 * link_length.at(1) * link_length.at(2) * c3 );

    ROS_INFO("c2: %f", c2);
    ROS_INFO("s2: %f", s2);
    ROS_INFO("c3: %f", c3);
    ROS_INFO("s3: %f", s3);
    ROS_INFO("p0: %f", p[0]);
    ROS_INFO("p1: %f", p[1]);
    ROS_INFO("p2: %f", p[2]);

    q[0] = atan2((sgn*p[1]), (sgn*p[0]));
    q[1] = atan2(s2, c2);
    q[2] = atan2(s3, c3);

    ROS_INFO("q0: %f", q[0]);
    ROS_INFO("q1: %f", q[1]);
    ROS_INFO("q2: %f", q[2]);
}

void planner_cartesian::inverse_diffkinematics(double pv[], double q[], double qv[]){
  double s1 = sin(q[0]);
  double c1 = cos(q[0]);
  double c2 = cos(q[1]);
  double s2 = sin(q[1]);
  double c3 = cos(q[2]);
  double s3 = sin(q[2]);

  double s23 = sin(q[1]+q[2]); 
  double c23 = cos(q[1]+q[2]);

  double a1 = link_length.at(0);
  double a2 = link_length.at(1);
  double a3 = link_length.at(2);

  double pv1 = pv[0];
  double pv2 = pv[1];
  double pv3 = pv[2];

  double detJ = a2*(a3*a3)*(c1*c1)*(c23*c23)*s2-(a2*a2)*a3*(c1*c1)*(c2*c2)*s23+a2*(a3*a3)*(c23*c23)*(s1*s1)*s2-(a2*a2)*a3*(c2*c2)*(s1*s1)*s23+(a2*a2)*a3*(c1*c1)*c2*c23*s2-a2*(a3*a3)*(c1*c1)*c2*c23*s23+(a2*a2)*a3*c2*c23*(s1*s1)*s2-a2*(a3*a3)*c2*c23*(s1*s1)*s23;
  if (fabs(detJ)<=1e-3){
    ROS_INFO("DETJ PROBLEM!");
    detJ = 1e-3;
  }

  qv[0] = (c1*pv2)/(a2*(c1*c1)*c2+a3*(c1*c1)*c23+a2*c2*(s1*s1)+a3*c23*(s1*s1))-(pv1*s1)/(a2*(c1*c1)*c2+a3*(c1*c1)*c23+a2*c2*(s1*s1)+a3*c23*(s1*s1));
  qv[1] = (pv3*s23)/(a2*c2*s23-a2*c23*s2)+(c1*c23*pv1)/(a2*(c1*c1)*c2*s23-a2*(c1*c1)*c23*s2+a2*c2*(s1*s1)*s23-a2*c23*(s1*s1)*s2)+(c23*pv2*s1)/(a2*(c1*c1)*c2*s23-a2*(c1*c1)*c23*s2+a2*c2*(s1*s1)*s23-a2*c23*(s1*s1)*s2);
  qv[2] = -(pv3*(a2*s2+a3*s23))/(a2*a3*c2*s23-a2*a3*c23*s2)-(c1*pv1*(a2*c2+a3*c23))/(a2*a3*(c1*c1)*c2*s23-a2*a3*(c1*c1)*c23*s2+a2*a3*c2*(s1*s1)*s23-a2*a3*c23*(s1*s1)*s2)-(pv2*s1*(a2*c2+a3*c23))/(a2*a3*(c1*c1)*c2*s23-a2*a3*(c1*c1)*c23*s2+a2*a3*c2*(s1*s1)*s23-a2*a3*c23*(s1*s1)*s2);
}

/*void planner_cartesian::inverse_diffkinematics(double pv[], double q[], double qv[]){
  double s1 = sin(q[0]);
  double c1 = cos(q[0]);
  double s12 = sin(q[0]+q[1]); 
  double c12 = cos(q[0]+q[1]);

  double detJ = link_length.at(0) * link_length.at(1) * s1;
  if (fabs(detJ)<=1e-3){
    detJ = 1e-3;
  }

  qv[0] = ((link_length.at(1) * c12) * pv[0] + (link_length.at(1) * s12) * pv[1]) / detJ;
  qv[1] = -((link_length.at(0) * c1 + link_length.at(1) * c12)* pv[0] + (link_length.at(0) * s1 + link_length.at(1) * s12) * pv[1]) / detJ;
}*/
