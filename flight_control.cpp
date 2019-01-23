#include "dji_sdk_demo/demo_flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include <time.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <ctime>

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
const int takeoff_wait = 0; // sek

float search_height = 15, pickup_height = 5;

int somefile_state = 0;
int Print_counter = 0;
int search_counter = 0, square_counter = 1;
int flight_state = 0, state = 1;
float xCmd, yCmd, zCmd, yaw;
float x, y, z;
bool found = false;
float square_size = 5;

float homex = 0, homey = 0;
float dropx = 0, dropy = 1;
float s_x = 100, s_y = 100; // recure point

float targetx = 90;//camera_cordinatex; //(subscribe to get this coordiante)
float targety = 70;//camera_cordinatey; //(subscribe to get this coordiante)
float parameterx = targetx - 1;
float parametery = targety;

ros::ServiceClient set_local_pos_reference;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlPosYawPub;
ros::Publisher ctrlBrakePub;

uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Point current_local_pos;

std::ofstream somefile;

Mission square_mission;

void flight_pub();
void flight_plan();
void rand_camera();

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;


  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber gpsSub  	= nh.subscribe("dji_sdk/gps_position", 10, &gps_callback);
  ros::Subscriber flightStatusSub = nh.subscribe("dji_sdk/flight_status", 10, &flight_status_callback);
  ros::Subscriber displayModeSub = nh.subscribe("dji_sdk/display_mode", 10, &display_mode_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 10, &local_position_callback);

  ROS_INFO("Update");

  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  ctrlBrakePub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
 
  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service     	= nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service  	= nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");
  set_local_pos_reference	= nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;
  start_time = ros::Time::now();

  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if (!set_local_position()) // We need this for height
  {
	ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
	return 1;
  }

  if(is_M100())
  {
	ROS_INFO("takeoff in %d",takeoff_wait);
	ros::Duration(takeoff_wait).sleep();
	ROS_INFO("Ready");
	ROS_INFO("M100 taking off!");
	takeoff_result = M100monitoredTakeoff();
  }

  if(takeoff_result)
  {
	x = s_x;
	y = s_y;
	z = search_height;
	yaw = 0;
	flight_state = 2;
	ROS_INFO("flight state: %d",flight_state);
  }

  ros::spin();
  return 0;
}
bool
M100monitoredTakeoff()
{
  ros::Time start_time = ros::Time::now();

  float home_altitude = current_gps.altitude;
  if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
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

  if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
  	current_gps.altitude - home_altitude < 1.0)
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
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  current_atti = msg->quaternion;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
  flight_plan();
}
void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}
bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
	return true;
  }

  return false;
}
bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
	ROS_ERROR("obtain control failed!");
	return false;
  }

  return true;
}
bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
	ROS_ERROR("takeoff_land fail");
	return false;
  }

  return true;
}
bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}
void
localOffsetFromGpsOffset(geometry_msgs::Vector3&  deltaNed,
                     	sensor_msgs::NavSatFix& target,
                     	sensor_msgs::NavSatFix& origin)
{
  double deltaLon = target.longitude - origin.longitude;
  double deltaLat = target.latitude - origin.latitude;

  deltaNed.y = deltaLat * deg2rad * C_EARTH;
  deltaNed.x = deltaLon * deg2rad * C_EARTH * cos(deg2rad*target.latitude);
  deltaNed.z = target.altitude - origin.altitude;
}

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}
void flight_pub()
{
  if (Print_counter == 50){
	if(somefile_state == 0){
  	somefile.open ("somefile.txt");
  	somefile_state = 1;}
	somefile << "current local pos x "; somefile << current_local_pos.x; somefile << " and latitude "; somefile << current_gps.latitude; somefile << "\n";
	somefile << "current local pos y "; somefile << current_local_pos.y; somefile << " and longitude "; somefile << current_gps.longitude; somefile << "\n";
	somefile << "current local pos z "; somefile << current_local_pos.z; somefile << " and altitude "; somefile << current_gps.altitude; somefile << "\n \n";
	if(somefile_state == 2){
  	somefile.close();}

	ROS_INFO("x %f and current local pos x %f and latitude %f",x,current_local_pos.x,current_gps.latitude);
	ROS_INFO("y %f and current local pos y %f and longitude %f",y,current_local_pos.y,current_gps.longitude);
	ROS_INFO("z %f and current local pos z %f and altitude %f",z,current_local_pos.z,current_gps.altitude);
	ROS_INFO("yaw: %f xCmd: %f yCmd: %f zCmd: %f",yaw,xCmd,yCmd,zCmd);
	Print_counter = 0;
  }
  sensor_msgs::Joy controlPosYaw;
  controlPosYaw.axes.push_back(xCmd);
  controlPosYaw.axes.push_back(yCmd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(yaw);
  ctrlPosYawPub.publish(controlPosYaw);
}
void flight_plan(){
  if ( current_local_pos.x > 0 && current_local_pos.y > 0 ){
  if ( abs(current_local_pos.x - targetx) < 5 && abs(current_local_pos.y - targety) < 5 ){
	found = true;
	ROS_INFO("True");
  }}
  switch(flight_state){
  case 0:
	if (square_counter != 1){
  	somefile_state = 2;
  	Print_counter = 50;
  	flight_pub();
  	ROS_INFO("all done");
	}
  break;
  case 1:
	if (found){

  	rand_camera();
/*
    	flight_state = 3;
    	ROS_INFO("flight state: %d",flight_state);
*/ 	 


	}else{
  	if(square_counter<10){
    	z = search_height;
  	switch(search_counter){
  	case 0:
    	x = s_x + square_size*square_counter;
    	if (square_counter == 1)
    	{
      	y = s_y;
    	}else{
      	y = s_y - square_size*(square_counter-1);
    	}
    	yaw = 0;
    	search_counter++;
    	flight_state = 2;
    	ROS_INFO("flight state: %d",flight_state);
  	break; 	 
  	case 1:
    	x = s_x + square_size*square_counter;
    	y = s_y + square_size*square_counter;
    	yaw = 90;
    	search_counter++;
    	flight_state = 2;
    	ROS_INFO("flight state: %d",flight_state);
  	break;
  	case 2:
    	x = s_x - square_size*square_counter;
    	y = s_y + square_size*square_counter;
    	yaw = 180;
    	search_counter++;
    	flight_state = 2;
    	ROS_INFO("flight state: %d",flight_state);
  	break;
  	case 3:
    	x = s_x - square_size*square_counter;
    	y = s_y - square_size*square_counter;
    	yaw = 270;
    	search_counter = 0;
    	square_counter++;
    	flight_state = 2;
    	ROS_INFO("counter: %d",square_counter);
    	ROS_INFO("flight state: %d",flight_state);
  	break;
  	}
  	}else{
    	flight_state = 0;
  	}
	}
  break;
  case 2:  
	xCmd = x-current_local_pos.x;
	yCmd = y-current_local_pos.y;
	zCmd = z;
	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
  	if (found){
    	rand_camera();
    	//flight_state = 3;
    	//ROS_INFO("flight state: %d",flight_state);
  	}else{
    	Print_counter++;
    	flight_pub();
  	}
	}else{
  	ros::Duration(2).sleep();
  	flight_state = 1;
  	ROS_INFO("flight state: %d",flight_state);
	}
  break;
  case 3:
	switch (state){
  	case 1: //Fly closer (1 meter from target)
    	x = parameterx;
    	y = parametery;
    	xCmd = x - current_local_pos.x;
    	yCmd = y - current_local_pos.y;
    	zCmd = search_height;
    	ROS_INFO("fly 1");
    	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
      	flight_pub();
    	}else{
      	ros::Duration(2).sleep();
      	state = 2;
      	ROS_INFO("state: %d",state);
    	}
  	break;
  	case 2:
    	x = parameterx;
    	y = parametery;
    	xCmd = x - current_local_pos.x;
    	yCmd = y - current_local_pos.y;
    	zCmd = pickup_height;
    	ROS_INFO("fly 2");
    	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
      	flight_pub();
      	ROS_INFO("Lowering net pickup");
      	ROS_INFO("xCmd: %f yCmd: %f zCmd: %f",xCmd,yCmd,zCmd);
    	}else{
      	ros::Duration(2).sleep();
      	state = 3;
      	ROS_INFO("state: %d",state);
    	}
  	break;
  	case 3:
    	x = targetx;
    	y = targety;
    	xCmd = x - current_local_pos.x;
    	yCmd = y - current_local_pos.y;
    	zCmd = pickup_height;
    	ROS_INFO("fly 3");
    	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
      	flight_pub();
    	}else{
      	ros::Duration(2).sleep();
      	state = 4;
      	ROS_INFO("state: %d",state);
    	}
  	break;
  	case 4:
    	x = targetx;
    	y = targety;
    	xCmd = x - current_local_pos.x;
    	yCmd = y - current_local_pos.y;
    	zCmd = search_height;
    	ROS_INFO("fly 4");
    	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
      	ROS_INFO("Reasing net pickup");
      	flight_pub();
    	}else{
      	ros::Duration(2).sleep();
      	flight_state = 4;
      	state = 1;
      	ROS_INFO("state: %d",state);
    	}
  	break;
	}
	break;
case 4:
  	x = dropx;
  	y = dropy;
  	xCmd = x - current_local_pos.x;
  	yCmd = y - current_local_pos.y;
  	zCmd = search_height;
  	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
    	flight_pub();
  	}else{
    	ros::Duration(2).sleep();
    	flight_state = 5;
    	ROS_INFO("state: %d",state);
  	}
	break;
case 5:
  	x = dropx;
  	y = dropy;
  	xCmd = x - current_local_pos.x;
  	yCmd = y - current_local_pos.y;
  	zCmd = 2;
  	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
    	flight_pub();
  	}else{
    	ROS_INFO("Lowering net");
    	ros::Duration(2).sleep();
    	ROS_INFO("Release net");
    	ros::Duration(2).sleep();
    	flight_state = 6;
    	ROS_INFO("state: %d",state);
  	}
	break;
case 6:
  	x = dropx;
  	y = dropy;
  	xCmd = x - current_local_pos.x;
  	yCmd = y - current_local_pos.y;
  	zCmd = search_height;
  	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
    	flight_pub();
  	}else{
    	ros::Duration(2).sleep();
    	flight_state = 7;
    	ROS_INFO("state: %d",state);
  	}
	break;
case 7:
  	x = homex;
  	y = homey;
  	xCmd = x - current_local_pos.x;
  	yCmd = y - current_local_pos.y;
  	zCmd = search_height;
  	if (abs(xCmd) > 0.5 || abs(yCmd) > 0.5  || abs(zCmd-current_local_pos.z) > 0.5 ){
    	flight_pub();
  	}else{
    	//land();
    	ros::Duration(2).sleep();
    	flight_state = 0;
    	ROS_INFO("state: %d",state);
  	}
	break;   
  break;
  }
}


void rand_camera(){
 
 
  //bool rand_found = false;
  int i=0;
 
  srand (time(NULL));

while(i<10){

  int rand_nr;
  rand_nr = rand() % 4;
  ROS_INFO("random number %d",rand_nr);
 

  switch(rand_nr){
	case 0 :
  	ROS_INFO("v");
  	xCmd = current_local_pos.x;
  	yCmd = current_local_pos.y;
  	zCmd = search_height;

  	yaw = yaw - 5;
  	flight_pub();
  	ROS_INFO("yaw: %f", yaw);

  	break;
	case 1 :
  	ROS_INFO("o");
  	break;
	case 2 :
  	ROS_INFO("h");
  	xCmd = current_local_pos.x;
  	yCmd = current_local_pos.y;
  	zCmd = search_height;
  	yaw = yaw + 5;
  	ROS_INFO("yaw: %f", yaw);
  	break;
	case 3 :
  	ROS_INFO("n");
  	break;  
    
  }
  i++;
}

  flight_state = 3;
  ROS_INFO("flight state: %d",flight_state);

}