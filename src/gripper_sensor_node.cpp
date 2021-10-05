#include <ros/ros.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction> GrabClient;
// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> ReleaseClient;


class Gripper{
private:
  GrabClient* grab_client_;
  ReleaseClient* release_client_;

public:
  //Action client initialization
  Gripper(){

    //Initialize the client for the Action interface to the gripper controller
    //and tell the action client that we want to spin a thread by default
    grab_client_  = new GrabClient("r_gripper_sensor_controller/grab",true);
    release_client_  = new ReleaseClient("r_gripper_sensor_controller/release",true);

    while(!grab_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/grab action server to come up");
    }

    while(!release_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the r_gripper_sensor_controller/release action server to come up");
    }
  }

  ~Gripper(){
    delete grab_client_;
    delete release_client_;
  }

  //Open the gripper, find contact on both fingers, and go into slip-servo control mode
  void grab(){
    pr2_gripper_sensor_msgs::PR2GripperGrabGoal grip;
    grip.command.hardness_gain = 0.03;
    
    ROS_INFO("Sending grab goal");
    grab_client_->sendGoal(grip);
    grab_client_->waitForResult(ros::Duration(20.0));
    if(grab_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Successfully completed Grab");
    else
      ROS_INFO("Grab Failed");
  }

  // Look for side impact, finerpad slip, or contact acceleration signals and release the object once these occur
  void release(){
    pr2_gripper_sensor_msgs::PR2GripperReleaseGoal place;
    // set the robot to release on a figner-side impact, fingerpad slip, or acceleration impact with hand/arm
    place.command.event.trigger_conditions = place.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;  
    // set the acceleration impact to trigger on to 5 m/s^2
    place.command.event.acceleration_trigger_magnitude = 5.0; 
    // set our slip-gain to release on to .005
    place.command.event.slip_trigger_magnitude = .01;


    ROS_INFO("Waiting for object placement contact...");
    release_client_->sendGoal(place);
    release_client_->waitForResult();
    if(release_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Release Success");
    else
      ROS_INFO("Place Failure");

  }


};

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_gripper");

  Gripper gripper;

  // wait a few seconds so we can put something in the robot's gripper
  sleep(5.0);

  // grab something
  gripper.grab();

  // move the robot arm all ove the place here!
  sleep(5.0);
  
  // now that we've decided we want to look for contact and let go
  gripper.release();
  

  return 0;
}
