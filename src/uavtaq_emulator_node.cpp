#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
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
	std::string gas_a_topic = "gas/a";
	std::string gas_b_topic = "gas/b";
	std::string gas_c_topic = "gas/c";

	//==== Get parameters if set ====//
	if( !nh.getParam( "image_topic", image_topic ) ) {
		ROS_WARN( "No parameter set for \"image_topic\", using: %s", image_topic.c_str() );
	} else {
		ROS_INFO( "Publishing image to: %s", image_topic.c_str() );
	}

	if( !nh.getParam( "transform_topic", transform_topic ) ) {
		ROS_WARN( "No parameter set for \"transform_topic\", using: %s", transform_topic.c_str() );
	} else {
		ROS_INFO( "Publishing transform to: %s", transform_topic.c_str() );
	}

	if( !nh.getParam( "gas_a_topic", gas_a_topic ) ) {
		ROS_WARN( "No parameter set for \"gas_a_topic\", using: %s", gas_a_topic.c_str() );
	} else {
		ROS_INFO( "Publishing gas data (a) to: %s", gas_a_topic.c_str() );
	}

	if( !nh.getParam( "gas_b_topic", gas_b_topic ) ) {
		ROS_WARN( "No parameter set for \"gas_b_topic\", using: %s", gas_b_topic.c_str() );
	} else {
		ROS_INFO( "Publishing gas data (b) to: %s", gas_b_topic.c_str() );
	}

	if( !nh.getParam( "gas_c_topic", gas_c_topic ) ) {
		ROS_WARN( "No parameter set for \"gas_c_topic\", using: %s", gas_c_topic.c_str() );
	} else {
		ROS_INFO( "Publishing gas data (c) to: %s", gas_c_topic.c_str() );
	}

	//==== Begin publisher & subscriber ====//
	ros::Publisher trans_pub = nh.advertise<geometry_msgs::TransformStamped>(transform_topic, 100);
	ros::Publisher gas_a_pub = nh.advertise<std_msgs::Int32>(gas_a_topic, 10);
	ros::Publisher gas_b_pub = nh.advertise<std_msgs::Int32>(gas_b_topic, 10);
	ros::Publisher gas_c_pub = nh.advertise<std_msgs::Int32>(gas_c_topic, 10);

	image_transport::ImageTransport it(nh);
	image_transport::Publisher img_pub = it.advertise(image_topic, 1);
	//cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	//cv::waitKey(30);

	//Loop
	ros::Rate loop_rate(100);
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
	double movementStep = 2*M_PI/numStep;
	int currentStep = 0;
	geometry_msgs::TransformStamped trans_out;
	trans_out.header.frame_id = "world";
	trans_out.child_frame_id = "uav";
	trans_out.transform.translation.z = 2;
	trans_out.transform.rotation.w = 0;
	trans_out.transform.rotation.x = 1;
	trans_out.transform.rotation.y = 0;
	trans_out.transform.rotation.z = 0;


	//==== Begin publishing topics ====//
	while ( ros::ok() ) {
		//Gas
		if( gas_rate_counter >= 20 ) {
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
		if( img_rate_counter >= 50 ) {
			img_seq++;
			img_out.header.stamp = ros::Time::now();
			img_out.header.seq = img_seq;

			img_pub.publish(img_out);

			img_rate_counter = 0;
		}

		img_rate_counter++;

		//Transform
		currentStep++;
		trans_seq++;

		if(currentStep > numStep) {
			currentStep = 0;
		}

		trans_out.header.stamp = ros::Time::now();
		trans_out.header.seq = trans_seq;

		trans_out.transform.translation.x = 2*std::cos(currentStep*movementStep);
		trans_out.transform.translation.y = 4*std::sin(currentStep*movementStep);

		trans_pub.publish(trans_out);

		//Sleep
		loop_rate.sleep();
	}

	return 0;
}

