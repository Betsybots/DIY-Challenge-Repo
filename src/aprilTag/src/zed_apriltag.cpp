// zed_apriltag.cpp : ROS2 Humble node that detects AprilTags on images from a
// separately launched ZED wrapper and publishes each tag's ID + label.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // Iron and newer
#else
#include <cv_bridge/cv_bridge.h>  // Humble
#endif

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <yaml-cpp/yaml.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "zed_apriltag/msg/tag_info.hpp"

class ZedAprilTagNode : public rclcpp::Node
{
public:
	ZedAprilTagNode()
		: Node("zed_apriltag_node")
	{
		// --- parameters ---
		tag_size_ = declare_parameter<double>("tag_size", 0.16);           // meters
		publish_image_ = declare_parameter<bool>("publish_debug_image", true);
		const std::string tag_family = declare_parameter<std::string>("tag_family", "tag36h11");
		const std::string image_topic =
			declare_parameter<std::string>("image_topic", "/zed/zed_node/rgb/color/rect/image");
		const std::string camera_info_topic =
			declare_parameter<std::string>("camera_info_topic", "/zed/zed_node/rgb/camera_info");
		const std::string labels_file = declare_parameter<std::string>("tag_labels_file", "");

		loadLabels(labels_file);

		// --- publishers ---
		detections_pub_ =
			create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("apriltag/detections", 10);
		tag_info_pub_ = create_publisher<zed_apriltag::msg::TagInfo>("apriltag/tag_info", 10);
		if (publish_image_) {
			image_pub_ = create_publisher<sensor_msgs::msg::Image>("apriltag/image", 10);
		}
		tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

		// --- AprilTag detector (OpenCV ArUco) ---
		detector_.setDictionary(cv::aruco::getPredefinedDictionary(familyFromString(tag_family)));

		// The ZED wrapper publishes with SensorDataQoS (best-effort).
		camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
			camera_info_topic, rclcpp::SensorDataQoS(),
			std::bind(&ZedAprilTagNode::onCameraInfo, this, std::placeholders::_1));
		image_sub_ = create_subscription<sensor_msgs::msg::Image>(
			image_topic, rclcpp::SensorDataQoS(),
			std::bind(&ZedAprilTagNode::onImage, this, std::placeholders::_1));

		RCLCPP_INFO(get_logger(),
					"zed_apriltag_node started (tag_size=%.3f m, family=%s, image=%s, %zu labels)",
					tag_size_, tag_family.c_str(), image_topic.c_str(), labels_.size());
	}

private:
	static cv::aruco::PredefinedDictionaryType familyFromString(const std::string& family)
	{
		if (family == "tag16h5")  return cv::aruco::DICT_APRILTAG_16h5;
		if (family == "tag25h9")  return cv::aruco::DICT_APRILTAG_25h9;
		if (family == "tag36h10") return cv::aruco::DICT_APRILTAG_36h10;
		return cv::aruco::DICT_APRILTAG_36h11; // default tag36h11
	}

	void loadLabels(const std::string& path)
	{
		if (path.empty()) {
			RCLCPP_WARN(get_logger(), "tag_labels_file not set; all tags will be labeled UNKNOWN");
			return;
		}
		YAML::Node root = YAML::LoadFile(path);
		const YAML::Node map = root["tag_labels"] ? root["tag_labels"] : root;
		if (!map.IsMap()) {
			throw std::runtime_error("tag_labels_file must contain a 'tag_labels' mapping");
		}
		for (const auto& entry : map) {
			std::string key = entry.first.as<std::string>();
			// Accept both "ID7" and plain "7" keys.
			if (key.rfind("ID", 0) == 0 || key.rfind("id", 0) == 0) {
				key = key.substr(2);
			}
			labels_[std::stoi(key)] = entry.second.as<std::string>();
		}
	}

	void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
	{
		camera_matrix_ = (cv::Mat_<double>(3, 3) <<
			msg->k[0], msg->k[1], msg->k[2],
			msg->k[3], msg->k[4], msg->k[5],
			msg->k[6], msg->k[7], msg->k[8]);
		dist_coeffs_ = cv::Mat(msg->d, true);  // rectified ZED image: zeros
		has_camera_info_ = true;
		camera_info_sub_.reset();  // intrinsics are static; one message is enough
	}

	void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
	{
		cv_bridge::CvImagePtr cv;
		try {
			cv = cv_bridge::toCvCopy(msg, "bgr8");
		} catch (const cv_bridge::Exception& e) {
			RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
			return;
		}

		cv::Mat gray;
		cv::cvtColor(cv->image, gray, cv::COLOR_BGR2GRAY);

		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f>> corners;
		detector_.detectMarkers(gray, corners, ids);

		apriltag_msgs::msg::AprilTagDetectionArray det_array;
		det_array.header = msg->header;

		const float s = static_cast<float>(tag_size_) / 2.0f;
		const std::vector<cv::Point3f> obj_pts = {
			{-s,  s, 0.f}, { s,  s, 0.f}, { s, -s, 0.f}, {-s, -s, 0.f}};

		for (size_t i = 0; i < ids.size(); ++i) {
			zed_apriltag::msg::TagInfo info;
			info.header = msg->header;
			info.id = ids[i];
			const auto it = labels_.find(ids[i]);
			info.label = (it != labels_.end()) ? it->second : "UNKNOWN";
			tag_info_pub_->publish(info);

			apriltag_msgs::msg::AprilTagDetection det;
			det.id = ids[i];
			for (int c = 0; c < 4; ++c) {
				det.corners[c].x = corners[i][c].x;
				det.corners[c].y = corners[i][c].y;
			}
			const cv::Point2f center =
				(corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]) * 0.25f;
			det.centre.x = center.x;
			det.centre.y = center.y;
			det_array.detections.push_back(det);

			// Pose/TF needs intrinsics from camera_info.
			if (has_camera_info_) {
				cv::Vec3d rvec, tvec;
				if (cv::solvePnP(obj_pts, corners[i], camera_matrix_, dist_coeffs_,
								 rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE)) {
					publishTransform(msg->header, ids[i], rvec, tvec);
				}
			}
		}

		detections_pub_->publish(det_array);

		if (publish_image_ && image_pub_->get_subscription_count() > 0) {
			if (!ids.empty()) {
				cv::aruco::drawDetectedMarkers(cv->image, corners, ids);
			}
			image_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", cv->image).toImageMsg());
		}
	}

	void publishTransform(const std_msgs::msg::Header& header, int id,
						  const cv::Vec3d& rvec, const cv::Vec3d& tvec)
	{
		cv::Mat R;
		cv::Rodrigues(rvec, R);

		// Convert rotation matrix to quaternion.
		const double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
		double qw, qx, qy, qz;
		if (trace > 0.0) {
			const double sdiv = std::sqrt(trace + 1.0) * 2.0;
			qw = 0.25 * sdiv;
			qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) / sdiv;
			qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) / sdiv;
			qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) / sdiv;
		} else if (R.at<double>(0, 0) > R.at<double>(1, 1) &&
				   R.at<double>(0, 0) > R.at<double>(2, 2)) {
			const double sdiv = std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2)) * 2.0;
			qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / sdiv;
			qx = 0.25 * sdiv;
			qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / sdiv;
			qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / sdiv;
		} else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
			const double sdiv = std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2)) * 2.0;
			qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / sdiv;
			qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / sdiv;
			qy = 0.25 * sdiv;
			qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / sdiv;
		} else {
			const double sdiv = std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1)) * 2.0;
			qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / sdiv;
			qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / sdiv;
			qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / sdiv;
			qz = 0.25 * sdiv;
		}

		geometry_msgs::msg::TransformStamped tf;
		tf.header = header;
		tf.child_frame_id = "tag_" + std::to_string(id);
		tf.transform.translation.x = tvec[0];
		tf.transform.translation.y = tvec[1];
		tf.transform.translation.z = tvec[2];
		tf.transform.rotation.w = qw;
		tf.transform.rotation.x = qx;
		tf.transform.rotation.y = qy;
		tf.transform.rotation.z = qz;
		tf_broadcaster_->sendTransform(tf);
	}

	// Detection
	cv::aruco::ArucoDetector detector_;
	double tag_size_{0.16};
	bool publish_image_{true};
	std::map<int, std::string> labels_;

	// Intrinsics from camera_info
	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;
	bool has_camera_info_{false};

	// ROS interfaces
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
	rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_pub_;
	rclcpp::Publisher<zed_apriltag::msg::TagInfo>::SharedPtr tag_info_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
	std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ZedAprilTagNode>());
	rclcpp::shutdown();
	return 0;
}

