//
//  main.cpp
//  waypoints_circular
//
//  Created by Harvard Virgil Humphrey on 2019/10/15.
//  Copyright © 2019 Harvard Virgil Humphrey. All rights reserved.
//

#include <iostream>
#include "circular.h"
#include "flightplan.h"
#include "dji_sdk/dji_sdk.h"
#include "dji_sdk/dji_sdk_node.h"

//Mathematicals
vector2D transform2D(vector2D input,double angle) { // Makeshift 2D coordinates transform by specified angle
    vector2D vector;
    vector.x = input.x*cos(angle)+input.y*sin(angle);
    vector.y = -input.x*sin(angle)+input.y*cos(angle);
    return vector;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

// Mission Constructor
mission::mission()  { // Assign the default values for the mission before getting the flightplan
    mission::qualisys_health = true;
    mission::mission_started = false;
    mission::waypoint_reached = false;
    mission::mission_complete = false;
    mission::phase_started = false;
    mission::phase = 0.0;
    mission::linger_time;
    mission::setTarget(0.0,0.0,0.0,0.0);
}

mission::~mission() {
    std::cout<<"Foobar 2"<<std::endl;
}

void mission::setTarget(double x,double y,double z,double theta) { // Reset the current waypoint target
    mission::target.x = x;
    mission::target.y = y;
    mission::target.z = z;
    mission::target.theta = theta;
    ROS_INFO_STREAM("New target has been set");
}

void mission::reset_phase()    { // Set all waypoint booleans to be as they were at the start of a mission phase
    mission::waypoint_reached = false;
    mission::phase_started = false;
    mission::phase++;
    mission::linger_time = ros::Time::now()-ros::Time::now();
    ROS_INFO_STREAM("Resetting phase...");
}

void mission::initiate_phase()   { // Change boolean to indicate phase execution is underway
    mission::phase_started = true;
    mission::phase_start = ros::Time::now();
    ROS_INFO_STREAM("Commencing phase...");
}

void mission::start_mission()    { // Change main boolean to indicate mission is underway (at the start of operation after takeoff)
    mission::mission_started = true;
    mission::phase++;
    //mission::phase_start = ros::Time::now();
    mission::mission_start = ros::Time::now();
    ROS_INFO_STREAM("Starting mission...");
}

void mission::evaluate()    { // Compare state with target to establish journey left to complete. Establish whether way point has been reached sufficiently to a specified tolerance
    //ROS_INFO_STREAM("Evaluating distance to target...");
    double norm_from_target;
    pose state;
    if(mission::qualisys_health)    {
        state.x = mission::qualisys_state.x; // For test purposes, do not use this yet
        state.y = mission::qualisys_state.y;
        state.z = mission::qualisys_state.z;
        state.theta = mission::qualisys_state.theta;
        //ROS_INFO_STREAM("Using qualisys state");
    }   else    {
        state.x = mission::gps_state.x;
        state.y = mission::gps_state.y;
        state.z = mission::gps_state.z;
        state.theta = mission::gps_state.theta;
        //ROS_INFO_STREAM("Using gps state");
    }
    mission::delta_state.x = mission::target.x-state.x;
    mission::delta_state.y = mission::target.y-state.y;
    mission::delta_state.z = mission::target.z-state.z;
    mission::delta_state.theta = mission::target.theta-state.theta;
    //ROS_INFO_STREAM("X: "<<state.x<<" Y: "<<state.y<<" Z: "<<state.z<<" Yaw: "<<state.theta);
    //ROS_INFO_STREAM(" State Y: "<<state.y);
    //ROS_INFO_STREAM("Distances: "<<mission::delta_state.x<<" "<<mission::delta_state.y<<" "<<mission::delta_state.z<<" "<<mission::delta_state.theta);
    //ROS_INFO_STREAM("Target: "<<mission::target.x<<" "<<mission::target.y<<" "<<mission::target.z<<" "<<mission::target.theta);
    norm_from_target = sqrt(delta_state.x*delta_state.x+delta_state.y*delta_state.y+delta_state.z*delta_state.z);
    //ROS_INFO_STREAM("Norm is: "<<norm_from_target);
    if(norm_from_target<0.1)   {
        //ROS_INFO_STREAM("Arrived around target...");
        if(!mission::waypoint_reached)   {
            mission::linger_start = ros::Time::now();
            ROS_INFO_STREAM("Lingering...");
        }
        mission::waypoint_reached = true;
        mission::linger_time = ros::Time::now()-mission::linger_start; // We want to hang around for 1 second before moving to the next waypoint
        ROS_INFO_STREAM("Phase number is: "<<mission::phase<<" out of "<<mission::nwaypoints<<" phases."<<" Linger time is "<<mission::linger_time.toSec());
        if(mission::phase == mission::nwaypoints && mission::linger_time.toSec()>1)    {
            ROS_INFO_STREAM("Final target reached, winding down mission... ");
            mission::mission_complete = true;
        }
    }   else    {
        if(mission::waypoint_reached)   {
            mission::waypoint_reached = false;
        }
    }
}

// Node Constructor
circular::circular() 
:nh_private_("~")
{
    // Get the flightplan into the mission class
    circular::nh_private_.param<double>("datum_angle",circular::datum_angle,0);
    circular::nh_private_.param<int>("n",circular::n,10);
    circular::nh_private_.param<double>("radius",circular::radius,1.5);
    circular::nh_private_.param<double>("target_height",circular::target_height,1.2);
    circular::nh_private_.param<double>("xygain",circular::gainstruct.xygain,0.25);
    circular::nh_private_.param<double>("vgain",circular::gainstruct.vgain,0.25);
    circular::nh_private_.param<double>("yawgain",circular::gainstruct.yawgain,0.5);
    //circular::circular_mission();
    //std::cout<<"n = "<<circular::n<<std::endl;
    circular::circular_mission.flightplan = circleflightplan(radius,n,target_height);
    // for(int i=0;i<n;i++)    {
    //     ROS_INFO_STREAM("Target X: "<<circular_mission.flightplan.waypts[i].x<<" Target Y: "<<circular_mission.flightplan.waypts[i].y<<" Target Z: "<<circular_mission.flightplan.waypts[i].z);
    // }
    circular::circular_mission.nwaypoints = circular::circular_mission.flightplan.nwaypoints;
    circular::m100.local_ref = false;
    ROS_INFO_STREAM("Flightplan compiled...");

    // ROS Services (ROS services for governing the aircraft (mostly guards))
    circular::sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
    circular::drone_arm_service = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
    circular::set_local_pos_reference = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
    circular::query_version_service = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
    circular::drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    // ROS Subscribers (All subscribers except for 1)
    circular::qualisysSub = nh.subscribe("/qualysis/matrice",10,&circular::mocap_pos_callback,this); // This will need changing later to match qualisys-accepted messages
    circular::gpsSub = nh.subscribe("/dji_sdk/gps_position",10,&circular::gps_callback,this);
    circular::attitudeSub = nh.subscribe("/dji_sdk/attitude",10,&circular::attitude_callback,this);
    circular::gps_healthSub = nh.subscribe("/dji_sdk/gps_health",10,&circular::gps_health_callback,this);
    circular::flight_statusSub = nh.subscribe("/dji_sdk/flight_status",10,&circular::flight_status_callback,this);
    // ROS Publisher
    circular::dji_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/flight_control_setpoint_generic", 10); // Publisher advertises messages for drone to take as control commands
    // ROS Timer
    circular::cmd_timer = nh.createTimer(ros::Duration(0.02), &circular::cmd_dji_callback,this); // Timer governing the rate at which publisher advertises messages

    // Calibrate the origin
    ROS_INFO_STREAM("First spinOnce");
    //ros::spinOnce(); // Activate all callback functions once to get state information
    ROS_INFO_STREAM("After first spinOnce");
    while(!circular::m100.local_ref)    {
        circular::m100.local_ref = circular::set_local_pos_ref(); // Use state information to establish the current position as origin
    }
    circular::local_posSub = nh.subscribe("/dji_sdk/local_position",10,&circular::local_pos_callback,this); // Activate local_sub for relative position from reference origin

    // Populate initial mission structs
    ros::Duration(0.005).sleep();
    //ROS_INFO_STREAM("Second spinOnce");
    //ros::spinOnce(); // Populate all state structures properly
    //ROS_INFO_STREAM("After second spinOnce");

    // Tell aircraft we're ready to fly
    
    circular::m100.in_control = circular::obtainCtrlAuthority();
    //circular::m100.motors_armed = circular::disArmMotors();
    //circular::m100.motors_armed = circular::armMotors();
    circular::m100.is_M100 = circular::is_M100();

    // Conduct the takeoff procedure
    if(circular::m100.is_M100)  {
        //ROS_INFO_STREAM("Attempting takeoff");
        circular::m100.airborne = M100monitoredTakeoff();
        //circular::m100.airborne = false;
        //circular::m100.airborne = circular::takeoff().result;
    }   else    {
        ROS_INFO_STREAM("Error! This is not a Matrice 100 aircraft. Shutting down now...");
        ros::shutdown();
    }
    //ROS_INFO_STREAM(circular::m100.airborne);
    //ros::Duration(20).sleep();
    // Begin the operation
    if(circular::m100.airborne) {
        circular::circular_mission.start_mission();
    }
    if(circular::n!=circular_mission.flightplan.waypts.size())  {
        ROS_INFO_STREAM("Flightplan error...");
        circular::circular_mission.mission_started = false;
    }
}

circular::~circular()   {
    std::cout<<"Foobar"<<std::endl;
    ros::shutdown();
}

// ROS Service Action definitions
ServiceAck circular::armMotors() { // Feeds power to the motors
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 1;
  circular::drone_arm_service.call(droneArmControl);
  if(!droneArmControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set, droneArmControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
    ROS_INFO_STREAM("Failed to arm motors");
  }
  return {droneArmControl.response.result, droneArmControl.response.cmd_set,
          droneArmControl.response.cmd_id, droneArmControl.response.ack_data};
}

ServiceAck circular::disArmMotors() { // Cuts of all command signals to the motors (does not necessarily shut them off)
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 0;
  circular::drone_arm_service.call(droneArmControl);
  if(!droneArmControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set, droneArmControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
    ROS_INFO_STREAM("Failed to disarm motors");
  }
  return {droneArmControl.response.result, droneArmControl.response.cmd_set,
          droneArmControl.response.cmd_id, droneArmControl.response.ack_data};
}

ServiceAck circular::obtainCtrlAuthority() { // Transfers control authority from Remote control to ROS Node
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  circular::sdk_ctrl_authority_service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    ROS_INFO_STREAM("Failed to obtain control authority");
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
          sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}

ServiceAck circular::releaseCtrlAuthority() { // Releases that authority
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 0;
  circular::sdk_ctrl_authority_service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    ROS_INFO_STREAM("Failed to release control authority");
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
          sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}

ServiceAck circular::takeoff() { // Instructs drone to take off. This should achieve a height of 1.2 m
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 4;
  circular::drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
    ROS_INFO_STREAM("Takeoff task failed");
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

ServiceAck circular::goHome() { // Land back at the origin 
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 1;
  circular::drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
    ROS_INFO_STREAM("Go Home task failed");
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

ServiceAck circular::land() { // Land where we are currently
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = 6;
  drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
    ROS_INFO_STREAM("Landing task failed");
  }
  return {droneTaskControl.response.result, droneTaskControl.response.cmd_set,
          droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data};
}

bool circular::set_local_pos_ref()   { // Activates ROS service establishing local reference point
    dji_sdk::SetLocalPosRef localPosReferenceSetter;
    set_local_pos_reference.call(localPosReferenceSetter);
    if(localPosReferenceSetter.response.result) {
        ROS_INFO_STREAM("Successful reference set...");
    }   else
    {
        ROS_INFO_STREAM("Haven't successfully established reference. Will keep trying");
    }
    
    return localPosReferenceSetter.response.result;
}

bool circular::is_M100()
{
  dji_sdk::QueryDroneVersion query;
  circular::query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }
  return false;
}

// ROS Subscriber Callback Functions

void circular::mocap_pos_callback(const qualisys::Subject::ConstPtr& msgs)   {
    circular::qualisys_msg = *msgs;
    double head1,head2,head3,head4;
        // Time reading
    circular::circular_mission.qualisys_state.header = msgs->header;
    circular::circular_mission.qualisys_state.x = msgs->position.x;
    circular::circular_mission.qualisys_state.y = msgs->position.y;
    circular::circular_mission.qualisys_state.z = msgs->position.z;
        
        // Orientation calculation
    head1 = msgs->orientation.x;
    head2 = msgs->orientation.y;
    head3 = msgs->orientation.z;
    head4 = msgs->orientation.w;
    tf::Quaternion q(head1,head2,head3,head4);
    tf::Matrix3x3 m(q);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    circular::circular_mission.qualisys_state.theta = fmod(yaw+2*M_PI,2*M_PI);
}

void circular::gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)    {
    circular::current_gps = *msg;
}

void circular::attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    circular::current_atti = msg->quaternion;
    circular::get_gps_heading(); // Must be dealt with
}

void circular::local_pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg)    {
    circular::current_ENU = *msg;
    if(circular::m100.local_ref)   {
        circular::circular_mission.gps_state.header = msg->header;
        circular::get_localgps_state(); // Must be dealt with
    }
}

void circular::flight_status_callback(const std_msgs::UInt8::ConstPtr& msg) {
    circular::m100.flight_status = msg->data;
}

void circular::gps_health_callback(const std_msgs::UInt8::ConstPtr& msg)   {
    circular::circular_mission.gps_health = msg->data;
}

// Publisher Callback Function
void circular::cmd_dji_callback(const ros::TimerEvent& event)   {
    if(circular::circular_mission.mission_started)  {
        circular::calculate_u(); // Calculates the necessary control command and stores this command in the m100 object
        sensor_msgs::Joy instruction;
        cmd_input& ref = circular::m100.control_input;
        instruction.axes.push_back(ref.EAST);
        instruction.axes.push_back(ref.NORTH);
        instruction.axes.push_back(ref.UP);
        instruction.axes.push_back(ref.yaw);
        instruction.axes.push_back(ref.flag);
        if(circular::circular_mission.mission_complete)    {
            ROS_INFO_STREAM("Mission completed. No more instructions will be published");
            circular::M100monitoredLand();
        }   else
        {
            circular::dji_pub.publish(instruction);
        }
    }
}

// State calculation functions
void circular::get_gps_heading()    { // Establishes the current heading in Euler angle relative to the M-Air facility datum angle
    double datum_angle = circular::circular_mission.datum_angle;
    circular::circular_mission.gps_state.theta = toEulerAngle(circular::current_atti).z-datum_angle;
}

void circular::get_localgps_state() { // Retrieves the local position in ENU Cartesian coordinates
    circular::circular_mission.gps_state.z = circular::current_ENU.point.z;
    vector2D ENU;
    ENU.x = circular::current_ENU.point.x;
    ENU.y = circular::current_ENU.point.y;
    //ROS_INFO_STREAM("Pre-transformed y: "<<ENU.y);
    vector2D transformed = transform2D(ENU,circular::datum_angle);
    //ROS_INFO_STREAM("Post-transformed y: "<<transformed.y);
    circular::circular_mission.gps_state.x = transformed.x;
    circular::circular_mission.gps_state.y = transformed.y;
}

void circular::get_mocap_health()   { // Evaluates the health of the qualisys signal based on time of previous message
    ros::Duration elapsed_time = circular::qualisys_msg.header.stamp-ros::Time::now();
    //ROS_INFO_STREAM("Qualisys elapsed time: "<<elapsed_time.toSec());
    if(elapsed_time.toSec()>(1.0/50) || elapsed_time.toSec()<0)   {
        circular::circular_mission.qualisys_health = false;
    }   else    {
        circular::circular_mission.qualisys_health = true;
    }
}

// Control input calculation
void circular::calculate_u()    { // Calculate the control command
    int mode;
    mode = 1;
    if(mode = 1)    { // Mode 1 is supposed to be static waypoints mode (only mode at the moment)
        if(circular::circular_mission.mission_complete) {
            return;
        }
        circular::get_mocap_health();
        if(circular::circular_mission.linger_time.toSec()>1)    {
                circular::circular_mission.reset_phase();
                vector3D new_target = circular::circular_mission.flightplan.waypts.at(circular::circular_mission.phase-1);
                circular::circular_mission.setTarget(new_target.x,new_target.y,new_target.z,0.0);
        }
        if(!circular::circular_mission.phase_started) {
            circular::circular_mission.initiate_phase();
            //std::cout<<circular_mission.flightplan.waypts.at(circular::circular_mission.phase).x<<" "<<circular_mission.flightplan.waypts.at(circular::circular_mission.phase).y<<" "<<circular_mission.flightplan.waypts.at(circular::circular_mission.phase).z<<std::endl;
        } 
        circular::circular_mission.evaluate(); //evaluate the error to the current waypoint
        vector2D origin;
        origin.x = circular::circular_mission.delta_state.x;
        origin.y = circular::circular_mission.delta_state.y;
        vector2D transformed = transform2D(origin,-1.0*circular::datum_angle); // transform the stuff back to the original coordinate system
        // Assign the stuff to the required struct in the m100 class
        circular::m100.control_input.EAST = circular::gainstruct.xygain*transformed.x;
        circular::m100.control_input.NORTH = circular::gainstruct.xygain*transformed.y;
        circular::m100.control_input.UP = circular::gainstruct.vgain*circular::circular_mission.delta_state.z;
        circular::m100.control_input.yaw = circular::gainstruct.yawgain*circular::circular_mission.delta_state.theta;
        circular::m100.control_input.flag = (DJISDK::VERTICAL_VELOCITY|DJISDK::HORIZONTAL_VELOCITY|DJISDK::YAW_RATE|DJISDK::HORIZONTAL_GROUND|DJISDK::STABLE_ENABLE);
        //ROS_INFO_STREAM("Commands: "<<m100.control_input.EAST<<" "<<m100.control_input.NORTH<<" "<<m100.control_input.UP<<" "<<m100.control_input.yaw<<" "<<m100.control_input.flag);
    }
}

// Initialisation operations

bool circular::M100monitoredTakeoff()   { // Do the takeoff
    ros::Time start_time = ros::Time::now();
    ros::spinOnce();
    //double home_altitude = circular::circular_mission.gps_state.z;
    ROS_INFO_STREAM("Altitude is: "<<circular::circular_mission.gps_state.z);
    if(!circular::takeoff().result) // May need investigating
    {
        return false;
    }
    ROS_INFO_STREAM("Takeoff request succeeded");
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    // Step 1: If M100 is not in the air after 20 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(20))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
        ROS_INFO_STREAM("Altitude is: "<<circular::circular_mission.gps_state.z);
    }
    //ROS_INFO_STREAM("Flying state should be: "<<DJISDK::M100FlightStatus::M100_STATUS_IN_AIR<<" It actually is: "<<circular::m100.flight_status);
    ROS_INFO_STREAM("Altitude is: "<<circular::circular_mission.gps_state.z);
    if(circular::m100.flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
        circular::circular_mission.gps_state.z < 0.8)
    {
        ROS_ERROR("Takeoff failed.");
        return false;
    }
    else
    {
        start_time = ros::Time::now();
        ROS_INFO("Successful takeoff!");
        ros::spinOnce();
    }
    return true;
}

bool circular::M100monitoredLand()   { // Do the takeoff
    ros::Time start_time = ros::Time::now();
    if(!circular::land().result) // May need investigating
    {
        return false;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
    // Step 1: If M100 is not in the air after 10 seconds, fail.
    while (ros::Time::now() - start_time < ros::Duration(10))
    {
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }
    if(circular::m100.flight_status == DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND ||
        circular::circular_mission.gps_state.z < 0.2)
    {
        ROS_INFO("Successful Landing...");
        ros::spinOnce();
    }
    else
    {
        ROS_ERROR("Landing failed.");
        return false;
    }
    return true;
}

int main(int argc, char ** argv) {
    // insert code here...
    ros::init(argc,argv,"Cirular_waypoints");
    ROS_INFO("ROS initialised... \n");
    circular Mission;
    ros::spin();
    ROS_INFO("Node initalised... \n");
    return 0;
}
