#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>

#include <string.h>
#include <math.h>
#include <cstdlib>

int main( int argc, char **argv ) {
	//==== Initialize node ====//
	ros::init( argc, argv, "uavtaq_emulator" );
	ros::NodeHandle nh( ros::this_node::getName() );

	//==== Set up variables ====//
	std::string image_topic = "image_raw";
	std::string transform_topic = "transform";
	std::string pose_topic = "transform";
	std::string grid_topic = "grid_map";
	std::string gas_a_topic = "gas/a";
	std::string gas_b_topic = "gas/b";
	std::string gas_c_topic = "gas/c";

	//==== Get parameters if set ====//
	nh.param( "topic_image", image_topic, image_topic );
	nh.param( "topic_transform", transform_topic, transform_topic );
	nh.param( "topic_pose", pose_topic, pose_topic );
	nh.param( "topic_occupancy_grid", grid_topic, grid_topic );
	nh.param( "topic_gas_a", gas_a_topic, gas_a_topic);
	nh.param( "topic_gas_b", gas_b_topic, gas_b_topic);
	nh.param( "topic_gas_c", gas_c_topic, gas_c_topic);

	//==== Begin publisher & subscriber ====//
	ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(transform_topic, 100);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>(pose_topic, 100);
	ros::Publisher gas_a_pub = nh.advertise<std_msgs::Int32>(gas_a_topic, 10);
	ros::Publisher gas_b_pub = nh.advertise<std_msgs::Int32>(gas_b_topic, 10);
	ros::Publisher gas_c_pub = nh.advertise<std_msgs::Int32>(gas_c_topic, 10);

	ros::Publisher grid_pub = nh.advertise<nav_msgs::OccupancyGrid>(grid_topic, 1, true);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher img_pub = it.advertise(image_topic, 1);

	//Loop Config
	ros::Rate loop_rate(50);

	//Gas
	int gas_rate_counter = 0;
	std_msgs::Int32 gas_a_out;
	std_msgs::Int32 gas_b_out;
	std_msgs::Int32 gas_c_out;

	//Image
	int img_seq = 0;
	int img_rate_counter = 0;
	int img_data_counter = 0;
	sensor_msgs::Image img_out;

	img_out.header.frame_id = "camera";
	img_out.height = 480;
	img_out.width = 640;
	img_out.encoding = "rgb8";
	img_out.is_bigendian = 0x00;
	img_out.step = 3*img_out.width;

	uint16_t c = 0xFF;
	bool flip = false;
	int ucount = 1;
	int vcount = 1;

	size_t st0 = (img_out.step * img_out.height);
	img_out.data.resize(st0);

	for(int v = 0; v < img_out.height; v++) {
		for(int u = 0; u < img_out.width; u++) {
			if( ucount < u ) {
				ucount = 2*ucount;

				if( flip ) {
					flip = false;
				} else {
					flip = true;
				}
			}
			if( flip ) {
				c = 0xFF;
			} else {
				c = 0x00;
			}

			img_out.data[img_data_counter] = c;
			img_data_counter++;
			img_out.data[img_data_counter] = c;
			img_data_counter++;
			img_out.data[img_data_counter] = c;
			img_data_counter++;
		}

		ucount = 1;

		if( vcount < v ) {
			vcount = 2*vcount;

			if( flip ) {
				flip = false;
			} else {
				flip = true;
			}
		}
	}

	//Transform
	int numStep = 1000;
	int trans_seq = 0;
	double movementStep = 2 * M_PI / numStep;
	int currentStep = 0;
	geometry_msgs::TransformStamped trans_out;
	trans_out.header.frame_id = "world";
	trans_out.child_frame_id = "uav";
	trans_out.transform.translation.z = 2;
	trans_out.transform.rotation.w = 0;
	trans_out.transform.rotation.x = 0;
	trans_out.transform.rotation.y = 0;
	trans_out.transform.rotation.z = 1;

	//Pose
	geometry_msgs::PoseStamped pose_out;
	pose_out.header.frame_id = trans_out.header.frame_id;
	pose_out.pose.position.z = trans_out.transform.translation.z;
	pose_out.pose.orientation.w = trans_out.transform.rotation.w;
	pose_out.pose.orientation.x = trans_out.transform.rotation.x;
	pose_out.pose.orientation.y = trans_out.transform.rotation.y;
	pose_out.pose.orientation.z = trans_out.transform.rotation.z;

	//Occupancy Grid
	std::srand( std::time(0) );
	ros::Time grid_stamp = ros::Time::now();
	nav_msgs::OccupancyGrid grid_out;
	grid_out.header.frame_id = "world";
	grid_out.header.stamp = grid_stamp;
	grid_out.info.map_load_time = grid_stamp;
	grid_out.info.resolution = 0.1;
	grid_out.info.width = 50;
	grid_out.info.height = 50;
	grid_out.info.origin.position.x = -2.5;
	grid_out.info.origin.position.y = -2.5;
	grid_out.info.origin.orientation.w = 1.0;

	for(int gv = 0; gv < ( grid_out.info.width * grid_out.info.height ); gv++) {
		int8_t val = 100;

		if( std::rand() % 8 ) {
			val = 0;
		}

		grid_out.data.push_back(val);
	}

	grid_pub.publish(grid_out);

	//==== Begin publishing topics ====//
	while ( ros::ok() ) {
		//Gas
		if( gas_rate_counter >= 10 ) {
			gas_a_out.data = std::rand() % 10 + 1;
			gas_b_out.data = std::rand() % 200 + 200;
			gas_c_out.data = std::rand() % 50 + 400;

			gas_a_pub.publish(gas_a_out);
			gas_b_pub.publish(gas_b_out);
			gas_c_pub.publish(gas_c_out);

			gas_rate_counter = 0;
		}

		gas_rate_counter++;

		//Image
		if( img_rate_counter >= 25 ) {
			img_out.header.stamp = ros::Time::now();
			img_out.header.seq = img_seq;

			img_pub.publish(img_out);

			img_rate_counter = 0;
		}

		img_rate_counter++;

		//Transform
		currentStep++;

		if(currentStep > numStep) {
			currentStep = 0;
		}

		trans_out.header.stamp = ros::Time::now();

		trans_out.transform.translation.x = 2 * std::cos( currentStep * movementStep );
		trans_out.transform.translation.y = 4 * std::sin( currentStep * movementStep );

		trans_pub.publish(trans_out);

		//Pose
		pose_out.header.stamp = ros::Time::now();

		pose_out.pose.position.x = trans_out.transform.translation.x;
		pose_out.pose.position.y = trans_out.transform.translation.y;

		pose_pub.publish(pose_out);

		//Sleep
		loop_rate.sleep();
	}

	return 0;
}

