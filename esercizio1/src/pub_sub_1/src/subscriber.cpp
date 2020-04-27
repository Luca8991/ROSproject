#include "ros/ros.h"
 
#define NAME_OF_THIS_NODE "subscriber"

#include "std_msgs/Float32.h"
 
//-----------------------------------------------------------------
//-----------------------------------------------------------------

class Subscriber
/* this class template includes all the elements needed to define
 * a basic ROS node */
{
  private: 
    ros::NodeHandle Handle;
    
    ros::Subscriber Subscriber;
     
    void MessageCallback(const std_msgs::Float32::ConstPtr& msg);
    
  public:
    
    void Prepare(void);
    
    void RunContinuously(void);
    
    void Shutdown(void);
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

void Subscriber::Prepare(void)
{

  Subscriber = Handle.subscribe("risultato", 1000, &Subscriber::MessageCallback, this);
   
  ROS_INFO("Node %s ready to run.", ros::this_node::getName().c_str());

}


void Subscriber::RunContinuously(void)
{

  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
   
  ros::spin();

}


void Subscriber::Shutdown(void)
{

  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());

}


void Subscriber::MessageCallback(const std_msgs::Float32::ConstPtr& msg)
{

  float risultato;

  risultato = msg->data;

  ROS_INFO("I've heard: [%f]", risultato);

}


//-----------------------------------------------------------------
//-----------------------------------------------------------------


int main(int argc, char **argv)
{
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  
  Subscriber MyNode;
   
  MyNode.Prepare();
  
  MyNode.RunContinuously();
   
  MyNode.Shutdown();
  
  return (0);
}
