#include "ros/ros.h"

#define RUN_PERIOD_DEFAULT 1
 
#define NAME_OF_THIS_NODE "publisher"

#include "std_msgs/Float32.h"
 
//-----------------------------------------------------------------
//-----------------------------------------------------------------

class Publisher
/* this class template includes all the elements needed to define
 * a basic ROS node */
{
  private: 
    ros::NodeHandle Handle;
    
    ros::Publisher Publisher;

    void PeriodicTask(int);
    
    float operazione(int);
    
  public:
    double RunPeriod;

    float a;  // x*y
    float b;  // x/y
    
    void Prepare(float, float);
    
    void RunPeriodically(float Period);
    
    void Shutdown(void);
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Publisher::Prepare(float x, float y)
{
  RunPeriod = RUN_PERIOD_DEFAULT;
  
  Publisher = Handle.advertise<std_msgs::Float32>("risultato", 1000);
  
  a = x * y;
  b = x / y;
   
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());
}

void Publisher::RunPeriodically(float Period)
{
  ros::Rate LoopRate(1.0/Period);
  
  ROS_INFO("Node %s running periodically (T=%.2fs, f=%.2fHz).", ros::this_node::getName().c_str(), Period, 1.0/Period);
  
  int t = 0;

  while (ros::ok())
  {
    PeriodicTask(t);

    t++;
     
    ros::spinOnce();
    /* From ROS documentation:
     * "ros::spinOnce() will call all the callbacks waiting to be
     * called at that point in time. ." */
    
    LoopRate.sleep();
  }
}


void Publisher::Shutdown(void)
{
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}


void Publisher::PeriodicTask(int t)
{
  std_msgs::Float32 msg;
  float risultato;

  risultato = operazione(t);

  msg.data = risultato;

  Publisher.publish(msg);

  ROS_INFO("I've published: '%f'", msg.data);
}


float Publisher::operazione(int t){
  float risultato;

  risultato = a * (t*t) + b * t;

  return risultato;
}

//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  Publisher MyNode;
   
  MyNode.Prepare(0.1, 0.2);
  
  MyNode.RunPeriodically(MyNode.RunPeriod);
   
  MyNode.Shutdown();
  
  return (0);
}