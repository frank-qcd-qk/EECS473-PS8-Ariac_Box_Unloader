#include <ros/ros.h>
#include <std_srvs/Trigger.h> // this message type is defined in the current package
void start_competition(ros::NodeHandle & startCompetitionNode){
    ros::ServiceClient startup_client = startCompetitionNode.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    if(!startup_client.exists()){
        ROS_INFO("[Startup Phase]Waiting for the competition to be ready...");
        startup_client.waitForExistence();
        ROS_INFO("[Startup Phase]Competition is now ready...");
    }
    ROS_INFO("[Startup Phase]Requesting competition to start...");
    std_srvs::Trigger startup_srv;
    startup_srv.response.success = false;
    startup_client.call(startup_srv);
    while(!startup_srv.response.success){
        ROS_WARN("[Startup Phase]Not successful startup yet... Repeat Calling...");
        startup_client.call(startup_srv);
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("[Startup Phase]Competition Startup Success>>>[Belt Control Phase]");
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "start_competition"); //Initialization of ros
    ros::NodeHandle n; //Creates RoS Node Handle
    //! Competition Startup Call
    start_competition(n);
}