#include <librealsense2/rs.hpp>
// #include <librealsense2/rs_advanced_mode.hpp>
#include <iostream>
#include <fstream>
#include <map>
#include <thread>
// #include <string>
#include <vector>
#include <string.h>
#include <stdio.h>
#include <sstream>
// include OpenCV header file
#include <opencv2/opencv.hpp>
#define localtime_r(_Time, _Tm)
#define localtime_s(_Tm, _Time)
#include <time.h>
// include NetworkPublisher and Dealer header files
#include "NetworkPublisher.h"
#include "Dealer.h"


using namespace rs2;
using namespace std;
using namespace cv;
using namespace attentivemachines;

time_t base_time = 0;
time_t current_time;
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Capture & save function
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

void capture(rs2::pipeline p, string serial, NetworkPublisher *np, Dealer *dealer)
{
    rs2::colorizer color_map;

    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

	cout << "\nStarting capture with the camera : " << serial << endl;
    for (auto i = 0; i < 30; ++i) p.wait_for_frames();
    while (true)
    {
	frameset fs = p.wait_for_frames();

    rs2::frameset aligned_set = align_to_color.process(fs);
    rs2::frame color_mat = aligned_set.get_color_frame();

    // for each frame get the Depth data and Colored data(RGB images) frame from the RealSense
    rs2::depth_frame depth_frame = fs.get_depth_frame();
    rs2::frame depth = fs.get_depth_frame().apply_filter(color_map);

    // Query frame size (width and height)
    const int w = depth.as<rs2::video_frame>().get_width();
    const int h = depth.as<rs2::video_frame>().get_height();

    // Create OpenCV matrix of size (w,h) from the colorized depth data
    Mat depth_image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);

    // Creating OpenCV Matrix from a color image(RGB images)
    Mat color(Size(848, 480), CV_8UC3, (void*)color_mat.get_data(), Mat::AUTO_STEP);

    // saving the RGB image as a vector
    std::vector<uchar> outputImage;
    try {
        bool ret = cv::imencode(".jpg", color, outputImage);
    }
    catch (std::exception ex) {
        std::cout << ex.what() << std::endl;
    }
    // creating a string wrapper for the RGB image
    std::string *str = new std::string(outputImage.begin(), outputImage.end());

    // Creating the RGB message with (the RGB image, [height and width are same as depth image] and compression type) as a Protobuf message
    PBImageSample* imageSample = new PBImageSample();
    imageSample->set_allocated_img(str);
    imageSample->set_width(google::protobuf::int32(depth_frame.get_width()));
    imageSample->set_height(google::protobuf::int32(depth_frame.get_height()));
    imageSample->set_compression(PBImageSample_Compression::PBImageSample_Compression_JPG);

    // saving the depth image as a vector
    std::vector<uchar> outputDepthImage;
    try {
        bool ret = cv::imencode(".png", depth_image, outputDepthImage);
    }
    catch (std::exception ex) {
        std::cout << ex.what() << std::endl;
    }
    // creating a string wrapper for the depth image
    str = new std::string(outputDepthImage.begin(), outputDepthImage.end());



    // Creating the Depth message with (the Depth image, height, width and compression type) as a Protobuf message
    PBDepthImageSample* depthImageSample = new PBDepthImageSample();
    depthImageSample->set_allocated_img(str);
    depthImageSample->set_width(google::protobuf::int32(depth_frame.get_width()));
    depthImageSample->set_height(google::protobuf::int32(depth_frame.get_height()));
    depthImageSample->set_compression(PBDepthImageSample_Compression::PBDepthImageSample_Compression_PNG);

    // Creating timestamp for the published messages
    tm localTime;
    std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
    time_t now = std::chrono::system_clock::to_time_t(t);
    localtime_r(&now, &localTime);

    const std::chrono::duration<double> tse = t.time_since_epoch();
    std::chrono::seconds::rep timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(tse).count();

    // cout << "Sensor " << serial << " time " << timestamp_ms << endl;

    stringstream geek(serial.substr(serial.size() - 4));
    int sourceID = 0;
    geek>>sourceID;

    // Creating a wrapper message for the Depth image to send it through the NetworkPublisher to the cnc-host
    PBMessage* message = np->NewMessage();
    message->set_timestamp(timestamp_ms);
    message->set_allocated_depthimage(depthImageSample);
    // publish the message
    np->MessageReady(message);

    // Creating a wrapper message for the RGB image to send it through the NetworkPublisher to the cnc-host
    PBMessage* msg = np->NewMessage();
    msg->set_timestamp(timestamp_ms);
    msg->set_allocated_image(imageSample);
    // publish the message
    np->MessageReady(msg);

    waitKey(10);

}

}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
// This function will go through every connected device (as long as it is a D415), apply a custom ".json", and then capture & save a ".png" for each device simultaneously.
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

int main(const int argc, const char **argv) try
{
    // inputing the path to config.ini file
    Config *config2 = CommonsTool::getConfig(argc, argv);
    // configuring the realsense client and the types of messages published
    config2->type = "realsense";
    config2->name = "RealsenseTable";
    config2->publishTypes.push_back(PBMessage::kImage);
    config2->publishTypes.push_back(PBMessage::kDepthImage);
    config2->sourceId = 2;
    CommonsTool::InitLogging(config2);

    // inputing the path to config.ini file
    Config *config1 = CommonsTool::getConfig(argc, argv);
    // configuring the realsense client and the types of messages published
    config1->type = "realsense";
    config1->name= "RealSenseCeiling";
    config1->publishTypes.push_back(PBMessage::kImage);
    config1->publishTypes.push_back(PBMessage::kDepthImage);
    config1->address = "tcp://127.0.0.1:6665";
    CommonsTool::InitLogging(config1);

    // Create a message publisher
    NetworkPublisher *np2 = new NetworkPublisher();
    np2->Initialize(config2);

    // Creating a dealer that connects to a configured cnc router instance and is responsible for
    // sending and receiving cnc messages
    Dealer *dealer2 = new Dealer();
    dealer2->Initialize(config2);

    // Create a message publisher
    NetworkPublisher *np1 = new NetworkPublisher();
    np1->Initialize(config1);

    // Creating a dealer that connects to a configured cnc router instance and is responsible for
    // sending and receiving cnc messages
    Dealer *dealer1 = new Dealer();
    dealer1->Initialize(config1);

	context ctx;
	vector<thread> threads;
	map<string, pipeline> pipes;

	for (auto dev : ctx.query_devices()) // For each device do :
	{
		if (strcmp(dev.get_info(RS2_CAMERA_INFO_NAME), "Intel RealSense D435") == 0) // Check for compatibility
		{
			rs2::pipeline p;
			rs2::config cfg;
			string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
			cout << "Configuring camera : " << serial << endl;

			// Add desired stream to configuration
            		cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 6);

            		// Add desired streams to configuration
            		cfg.enable_stream(RS2_STREAM_COLOR, 848, 480, RS2_FORMAT_BGR8, 6);


			cfg.enable_device(serial);

			// Start the pipeline
			p.start(cfg);
			cout << "Starting pipeline for current camera " << serial << endl;
			pipes[serial] = p;
		}
	}

    np2->Start();
    dealer2->Start();
    np1->Start();
    dealer1->Start();

	// Call the Capture & save function to start simultaneous threads
	for (map<string, pipeline>::iterator it = pipes.begin(); it != pipes.end(); ++it)
	{
        stringstream geek( it->first.substr( it->first.size() - 4));
        int sourceID = 0;
        geek>>sourceID;
        std::cout << sourceID << " " << it->first << endl;
        if (sourceID  == 848) {
            threads.push_back(std::thread(capture, it->second, it->first, np2, dealer2));
        }else{
            threads.push_back(std::thread(capture, it->second, it->first, np1, dealer1));
        }
	}

	// Joining the threads
	for (auto& t : threads)
	{
		t.join();
	}
    // close connection
    dealer2->Stop();
    np2->Stop();
    dealer1->Stop();
    np1->Stop();
    return EXIT_SUCCESS;
}

catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}

catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	system("pause");
	return EXIT_FAILURE;
}
