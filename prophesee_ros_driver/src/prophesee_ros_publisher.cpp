/*******************************************************************
 * File : prophesee_ros_publisher.cpp                              *
 *                                                                 *
 * Copyright: (c) 2015-2019 Prophesee                              *
 *******************************************************************/

#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>

#include "prophesee_ros_publisher.h"

#include <boost/date_time/posix_time/posix_time.hpp>

PropheseeWrapperPublisher::PropheseeWrapperPublisher() :
    nh_("~"),
    biases_file_(""),
    calibration_file_(""),
    raw_file_to_read_(""),
    activity_filter_temporal_depth_(0) {
    camera_name_ = "PropheseeCamera_optical_frame";

    // Load Parameters
    nh_.getParam("camera_name", camera_name_);
    nh_.getParam("publish_cd", publish_cd_);
    nh_.getParam("bias_file", biases_file_);
    nh_.getParam("instrinsics_calibration", calibration_file_);
    nh_.getParam("raw_file_to_read", raw_file_to_read_);
    nh_.getParam("activity_filter_temporal_depth", activity_filter_temporal_depth_);

    const std::string topic_cam_info        = "/prophesee/" + camera_name_ + "/camera_info";
    const std::string topic_cd_event_buffer = "/prophesee/" + camera_name_ + "/cd_events_buffer";

    pub_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_cam_info, 1);

    if (publish_cd_)
        pub_cd_events_ = nh_.advertise<prophesee_event_msgs::EventArray>(topic_cd_event_buffer, 500);

    while (!openCamera()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ROS_INFO("Trying to open camera...");
    }

    // Add camera runtime error callback
    camera_.add_runtime_error_callback([](const Metavision::CameraException &e) { ROS_WARN("%s", e.what()); });

    // Get the sensor config
    Metavision::CameraConfiguration config = camera_.get_camera_configuration();
    auto &geometry                        = camera_.geometry();
    ROS_INFO("[CONF] Width:%i, Height:%i", geometry.width(), geometry.height());
    ROS_INFO("[CONF] Activity Filter Temporal depth: %d [microseconds]", this->activity_filter_temporal_depth_);
    ROS_INFO("[CONF] Serial number: %s", config.serial_number.c_str());

    // Publish camera info message
    cam_info_msg_.width           = geometry.width();
    cam_info_msg_.height          = geometry.height();
    cam_info_msg_.header.frame_id = camera_name_;
    
    try {
        if (!calibration_file_.empty()) {
            ROS_INFO("[CONF] Loading instrinsics calibration file: %s", calibration_file_.c_str());
            set_intrinsics_calibration_from_file(calibration_file_);
            std::cout<<cam_info_msg_<<std::endl;


        } 
    } catch (Metavision::CameraException &e) { ROS_WARN("%s", e.what()); }

    // Set the activity filter instance
    if (activity_filter_temporal_depth_ > 0) {
        activity_filter_.reset(new Metavision::ActivityNoiseFilterAlgorithm<>(
            camera_.geometry().width(), camera_.geometry().height(), activity_filter_temporal_depth_));
    }
}

PropheseeWrapperPublisher::~PropheseeWrapperPublisher() {
    camera_.stop();

    nh_.shutdown();

    activity_filter_.reset();
}

bool PropheseeWrapperPublisher::openCamera() {
    bool camera_is_opened = false;

    // Initialize the camera instance
    try {
        if (raw_file_to_read_.empty()) {
            camera_ = Metavision::Camera::from_first_available();

            if (!biases_file_.empty()) {
                ROS_INFO("[CONF] Loading bias file: %s", biases_file_.c_str());
                camera_.biases().set_from_file(biases_file_);
            }
        } else {
            camera_ = Metavision::Camera::from_file(raw_file_to_read_);
            ROS_INFO("[CONF] Reading from raw file: %s", raw_file_to_read_.c_str());
        }

        camera_is_opened = true;
    } catch (Metavision::CameraException &e) { ROS_WARN("%s", e.what()); }
    return camera_is_opened;
}

bool PropheseeWrapperPublisher::set_intrinsics_calibration_from_file(std::string calibration_file_){
    std::ifstream file;
    file.open(calibration_file_);
    std::string buffer;
    std::string line;

    int nr_of_open = 0;
    while (std::getline(file, line)) {
        if (line.size() < 1) continue;

        nr_of_open += line.find("{") == -1;
        nr_of_open -= line.find("}") == -1;

        buffer += line;
        
        if (nr_of_open == 0){
            json_object *obj = json_tokener_parse(buffer.c_str());
            json_object *section;
            if (json_object_object_get_ex(obj, "camera_matrix", &section)) {
                
                json_object *uri;
                json_object_object_get_ex(section, "data", &uri);
                for (int i = 0; i < 9; i++) {
                    json_object *nr = json_object_array_get_idx(uri, i);
                    double number = json_object_get_double(nr);
                    cam_info_msg_.K[i] = number; 
                }
            }
            cam_info_msg_.D = {0,0,0,0,0};
            if (json_object_object_get_ex(obj, "distortion_coefficients", &section)) {
                
                json_object *uri;
                json_object_object_get_ex(section, "data", &uri);
                for (int i = 0; i < 5; i++) {
                    json_object *nr = json_object_array_get_idx(uri, i);
                    double number = json_object_get_double(nr);
                    cam_info_msg_.D[i] = number; 
                }
            }
        }
        
    }
    file.close();
   
    cam_info_msg_.P = {
        cam_info_msg_.K[0], cam_info_msg_.K[1], cam_info_msg_.K[2], 0,
        cam_info_msg_.K[3], cam_info_msg_.K[4], cam_info_msg_.K[5], 0,
        cam_info_msg_.K[6], cam_info_msg_.K[7], cam_info_msg_.K[8], 0};

    cam_info_msg_.R = {1,0,0,
                       0,1,0,
                       0,0,1};

    return true;
}

void PropheseeWrapperPublisher::startPublishing() {
    camera_.start();
    start_timestamp_ = ros::Time::now();
    last_timestamp_  = start_timestamp_;

    if (publish_cd_)
        publishCDEvents();

    ros::Rate loop_rate(5);
    while (ros::ok()) {
        if (pub_info_.getNumSubscribers() > 0) {
            /** Get and publish camera info **/
            cam_info_msg_.header.stamp = ros::Time::now();
            pub_info_.publish(cam_info_msg_);
        }
        loop_rate.sleep();
    }
}

void PropheseeWrapperPublisher::publishCDEvents() {
    // Initialize and publish a buffer of CD events
    try {
        Metavision::CallbackId cd_callback =
            camera_.cd().add_callback([this](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
                // Check the number of subscribers to the topic
                if (pub_cd_events_.getNumSubscribers() <= 0)
                    return;

                if (ev_begin < ev_end) {
                    // Compute the current local buffer size with new CD events
                    const unsigned int buffer_size = ev_end - ev_begin;

                    // Get the current time
                    event_buffer_current_time_.fromNSec(start_timestamp_.toNSec() + (ev_begin->t * 1000.00));

                    /** In case the buffer is empty we set the starting time stamp **/
                    if (event_buffer_.empty()) {
                        // Get starting time
                        event_buffer_start_time_ = event_buffer_current_time_;
                    }

                    /** Insert the events to the buffer **/
                    auto inserter = std::back_inserter(event_buffer_);

                    if (activity_filter_temporal_depth_ > 0) {
                        /** When there is activity filter **/
                        activity_filter_->process(ev_begin, ev_end, inserter);
                    } else {
                        /** When there is not activity filter **/
                        std::copy(ev_begin, ev_end, inserter);
                    }

                    /** Get the last time stamp **/
                    event_buffer_current_time_.fromNSec(start_timestamp_.toNSec() + (ev_end - 1)->t * 1000.00);
                }

                if ((event_buffer_current_time_ - event_buffer_start_time_) >= event_delta_t_) {
                    /** Create the message **/
                    prophesee_event_msgs::EventArray event_buffer_msg;

                    // Sensor geometry in header of the message
                    event_buffer_msg.header.stamp = event_buffer_current_time_;
                    event_buffer_msg.header.frame_id = camera_name_;
                    event_buffer_msg.height       = camera_.geometry().height();
                    event_buffer_msg.width        = camera_.geometry().width();

                    /** Set the buffer size for the msg **/
                    event_buffer_msg.events.resize(event_buffer_.size());

                    // Copy the events to the ros buffer format
                    auto buffer_msg_it = event_buffer_msg.events.begin();
                    for (const Metavision::EventCD *it = std::addressof(event_buffer_[0]);
                         it != std::addressof(event_buffer_[event_buffer_.size()]); ++it, ++buffer_msg_it) {
                        prophesee_event_msgs::Event &event = *buffer_msg_it;
                        event.x                            = it->x;
                        event.y                            = it->y;
                        event.polarity                     = it->p;
                        event.ts.fromNSec(start_timestamp_.toNSec() + (it->t * 1000.00));
                    }

                    // Publish the message
                    pub_cd_events_.publish(event_buffer_msg);

                    // Clean the buffer for the next itteration
                    event_buffer_.clear();

                    ROS_DEBUG("CD data available, buffer size: %d at time: %lui",
                              static_cast<int>(event_buffer_msg.events.size()), event_buffer_msg.header.stamp.toNSec());
                }

            });
    } catch (Metavision::CameraException &e) {
        ROS_WARN("%s", e.what());
        publish_cd_ = false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "prophesee_ros_publisher");

    PropheseeWrapperPublisher wp;
    wp.startPublishing();

    ros::shutdown();

    return 0;
}
