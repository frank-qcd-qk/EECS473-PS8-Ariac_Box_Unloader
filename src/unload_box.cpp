// unload_box.cpp:
// moves a box under camera, then removes sensed parts from box

// use a RobotBehaviorInterface object to communicate with the robot behavior
// action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

// we define a message type for a "part" that includes name, pose and associated
// location code (e.g. bin # or box location)
#include <inventory_msgs/Part.h>

// a "box inspector" object can compare a packing list to a logical camera image
// to see how we are doing
#include <box_inspector/box_inspector.h>

// conveyor interface communicates with the conveyor action server
#include <conveyor_as/ConveyorInterface.h>

const double COMPETITION_TIMEOUT =
    500.0;  // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

void model_to_part(osrf_gear::Model model, inventory_msgs::Part& part,
                   unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location;  // by default
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "box_unloader");  // node name
    ros::NodeHandle
        nh;  // create a node handle; need to pass this to the class constructor
    int ans;

    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(
        &nh);  // shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);

    ROS_INFO("instantiating a BoxInspector");
    BoxInspector boxInspector(&nh);

    // instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part;

    geometry_msgs::PoseStamped
        box_pose_wrt_world;  // camera sees box, coordinates are converted to
                             // world coords

    bool status;
    int nparts;

    // for box inspector, need to define multiple vectors for args,
    // box inspector will identify parts and convert their coords to world frame
    // in the present example, desired_models_wrt_world is left empty, so ALL
    // observed parts will be considered "orphaned"
    vector<osrf_gear::Model> desired_models_wrt_world;
    vector<osrf_gear::Model> satisfied_models_wrt_world;
    vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
    vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
    vector<osrf_gear::Model> missing_models_wrt_world;
    vector<osrf_gear::Model> orphan_models_wrt_world;
    vector<int> part_indices_missing;
    vector<int> part_indices_misplaced;
    vector<int> part_indices_precisely_placed;

    // use conveyor action  server for multi-tasking
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface
        .move_new_box_to_Q1();  // member function of conveyor interface to move
                                // a box to inspection station 1
    while (conveyorInterface.get_box_status() !=
           conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    // update box pose,  if possible
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    } else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
    }

    //! if survive to here, then box is at Q1 inspection station;
    //! Begin in box processing:
    ROS_INFO("[BOX handle] Begin Box Process!");
    /* The logic here is changed in order for a better result:
        Since orphaned parts are always going to be need to removed, it won't
       hurt to remove the orphaned parts first. Once orphaned parts are removed,
       we can start processing the parts that are supposed to be in the box by
       doing this, we can avoid robot arm hit any currently precise parts
    */


    //! Orphan handler:
    //TODO: get current status, prep for orphan
    boxInspector.update_inspection(
        desired_models_wrt_world, satisfied_models_wrt_world,
        misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
        orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
        part_indices_precisely_placed);
    //TODO: Ditch loop:
    while (orphan_models_wrt_world.size() > 0) {
        // TODO: Start removing orphaned parts first:
        ROS_INFO("[BOX handle orphan] Removing orphaned parts in box: ");
        nparts = orphan_models_wrt_world.size();
        ROS_INFO("[BOX handle orphan] num parts orphaned seen in box = %d",
                 nparts);
        for (int i = 0; i < nparts; i++) {
            model_to_part(orphan_models_wrt_world[i], current_part,
                          inventory_msgs::Part::QUALITY_SENSOR_1);
            ROS_INFO_STREAM("[BOX handle orphan] Removing orphaned parts: "
                            << orphan_models_wrt_world[i] << endl);

            status = robotBehaviorInterface.pick_part_from_box(current_part);
            if (status == false) {
                ROS_FATAL(
                    "[BOX Handle orphan]Critical issue: path plan failed");
            }
        }

        // TODO: Inspect the box one more time...
        boxInspector.update_inspection(
            desired_models_wrt_world, satisfied_models_wrt_world,
            misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing,
            part_indices_misplaced, part_indices_precisely_placed);
    }
    ROS_INFO("[BOX handle] All orphaned parts have been removed!");

    //! Bad part Handler:
    //TODO: get current status, prep for orphan
    boxInspector.update_inspection(
        desired_models_wrt_world, satisfied_models_wrt_world,
        misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
        orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
        part_indices_precisely_placed);


    if (boxInspector.get_bad_part_Q1(current_part)) {
        // TODO: Start removal of broken parts:
        ROS_INFO("[BOX handle] found bad part: ");
        ROS_INFO_STREAM(current_part << endl);
        status = robotBehaviorInterface.pick_part_from_box(current_part);
    }

    //! Rest of the part handler:
    // TODO: Inspect the parts left:
    boxInspector.update_inspection(
        desired_models_wrt_world, satisfied_models_wrt_world,
        misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
        orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
        part_indices_precisely_placed);
    //TODO: Ditch loop:
    while (satisfied_models_wrt_world.size() > 0) {
        // TODO: Ditch all the parts even they are good:
        ROS_INFO("[BOX handle good] Removing orphaned parts in box: ");
        nparts = satisfied_models_wrt_world.size();
        ROS_INFO("[BOX handle good] num parts orphaned seen in box = %d",
                 nparts);
        for (int i = 0; i < nparts; i++) {
            model_to_part(satisfied_models_wrt_world[i], current_part,
                          inventory_msgs::Part::QUALITY_SENSOR_1);
            status = robotBehaviorInterface.pick_part_from_box(current_part);
            if (status == false) {
                ROS_FATAL("[BOX Handle goo]Critical issue: path plan failed");
            }
            // TODO: Inspect the box one more time...
            boxInspector.update_inspection(
                desired_models_wrt_world, satisfied_models_wrt_world,
                misplaced_models_actual_coords_wrt_world,
                misplaced_models_desired_coords_wrt_world,
                missing_models_wrt_world, orphan_models_wrt_world,
                part_indices_missing, part_indices_misplaced,
                part_indices_precisely_placed);
        }
        return 0;
    }

    // TODO: All complete flag:
    ROS_INFO("[BOX handle all] All good parts have been removed!");
}
