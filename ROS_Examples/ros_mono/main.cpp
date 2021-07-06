#include <iostream>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include "glog/logging.h"

#include "opencv2/imgproc.hpp"

#include "QtVisualizer.h"
#include "FullSystem/FullSystem.h"
#include "Utils/Undistorter.h"

// Use the best GPU available for rendering (visualization)
#ifdef WIN32
extern "C"
{
	__declspec(dllexport) uint32_t NvOptimusEnablement = 0x00000001;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace dsm
{
  class RosProcessor {
    public:
      inline RosProcessor() : DSM_(nullptr) {};

      inline ~RosProcessor() {}

      inline void Init(Undistorter* undistorter,
                       QtVisualizer* visualizer,
                       const std::string& settingsFile) {
        setting_file_ = settingsFile;
        undistorter_ = undistorter;
        visualizer_ = visualizer;
      }

      inline void doRun(const sensor_msgs::ImageConstPtr& img_msg) {
        static int id = 0;
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img_msg);
        cv::Mat image = cv_ptr->image;
        double timestamp = cv_ptr->header.stamp.toSec();
        const double fps = 25.0;

        const cv::Mat& cvK = undistorter_->getK();
			  const Eigen::Matrix3f K((Eigen::Matrix3f() << cvK.at<double>(0, 0), cvK.at<double>(0, 1), cvK.at<double>(0, 2),
				                                              cvK.at<double>(1, 0), cvK.at<double>(1, 1), cvK.at<double>(1, 2),
				                                              cvK.at<double>(2, 0), cvK.at<double>(2, 1), cvK.at<double>(2, 2)).finished());

        // reset
				if (visualizer_->getDoReset()) {
					// reset slam system
					DSM_.reset();

					// reset variables
					id = 0;
					timestamp = 0;
					image.release();

					// reset visualizer
					visualizer_->reset();
				}

        // capture
				if (visualizer_->getDoProcessing()) {
					double time = (double)cv::getTickCount();

					//gray image from source
					if (image.channels() == 3) {
						cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
					}

					// undistort
					undistorter_->undistort(image, image);

					if (DSM_ == nullptr) {
						DSM_ = std::make_unique<FullSystem>(undistorter_->getOutputWidth(),
														   undistorter_->getOutputHeight(),
														   K, setting_file_,
														   visualizer_);
					}

					// process
					DSM_->trackFrame(id, timestamp, image.data);

					// visualize image
					visualizer_->publishLiveFrame(image);

					// increase counter
					++id;

					// wait
					time = 1000.0*(cv::getTickCount() - time) / cv::getTickFrequency();
					const double delay = (1000.0 / fps) - time;

					if (delay > 0.0) {
						std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(delay)));
					}
				} else {
					// allow other threads to run
					std::this_thread::yield();
				}

        // print log
        if (DSM_) {
          DSM_->printLog();
        }
        visualizer_->run(false);
      }

    private:
      std::unique_ptr<FullSystem> DSM_;
      std::string setting_file_;
      Undistorter* undistorter_;
      QtVisualizer* visualizer_;
  };

} // namespace dsm

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_mono");
  ros::start();

	// input arguments
	std::string imageTopic, calibFile, settingsFile;

	// Configuration
	if (argc == 4)
	{
		imageTopic = argv[1];
		calibFile = argv[2];
		settingsFile = argv[3];
	}
	else if (argc == 3)
	{
		imageTopic = argv[1];
		calibFile = argv[2];
	}
	else
	{
		std::cout << "The RosExample requires at least 2 arguments: imageTopic, calibFile, settingsFile (optional)\n";
		return 0;
	}

	// Initialize logging
	google::InitGoogleLogging(argv[0]);
	//FLAGS_logtostderr = 1;
	//FLAGS_v = 9;

	std::cout << "\n";

	// Create the application before the window always!
	// create visualizer in the main thread
	QApplication app(argc, argv);
	dsm::QtVisualizer visualizer(app);

	std::cout << "\n";

	// read calibration
	dsm::Undistorter undistorter(calibFile);
	if (!undistorter.isValid())
	{
		std::cout << "need camera calibration file..." << std::endl;
		return 0;
	}

	std::cout << "\n";

	if (imageTopic.empty())
	{
		std::cout << "image topic is empty..." << std::endl;
		return 0;
	}

	std::cout << "\n";

	// add image size to the visualizer
	visualizer.setImageSize(undistorter.getOutputWidth(), undistorter.getOutputHeight());

	// run processing in a second thread
	dsm::RosProcessor ros_processor;
	ros_processor.Init(&undistorter, &visualizer, settingsFile);

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe(imageTopic, 1, &dsm::RosProcessor::doRun, &ros_processor);

	// run main window
	visualizer.run(false);

  ros::spin();

	std::cout << "Finished!" << std::endl;

	return 1;
}