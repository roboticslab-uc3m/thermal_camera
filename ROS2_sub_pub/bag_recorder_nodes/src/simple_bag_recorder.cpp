#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <bits/stdc++.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>

using namespace std::chrono_literals;

using std::placeholders::_1;

std::string data_name = "";

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    // save the data_name to storage the bag info
    this->declare_parameter<std::string>("data_name", "");
    this->get_parameter("data_name", data_name);
    // change the name format
    std::string directory = "data_saved/" + data_name; 
    char *directory_array = new char[ directory.length() + 1];
    strcpy(directory_array, directory.c_str());
    // create the folder
    mkdir(directory_array, 0777);

    // create the writer
    const rosbag2_cpp::StorageOptions storage_options({"data_saved/"+ data_name, "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
      {rmw_get_serialization_format(),
       rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
      {"/teraranger_evo_thermal/raw_temp_array",
       "std_msgs/msg/Float64MultiArray",
       rmw_get_serialization_format(),
       ""});

    // subscribe to the topic
    subscription_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/teraranger_evo_thermal/raw_temp_array", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  // variables to store the data_name
  std::string parameter_string_;
  rclcpp::TimerBase::SharedPtr timer_;

  // callback to rosbag record
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      new rcutils_uint8_array_t,
      [this](rcutils_uint8_array_t *msg) {
        auto fini_return = rcutils_uint8_array_fini(msg);
        delete msg;
        if (fini_return != RCUTILS_RET_OK) {
          RCLCPP_ERROR(get_logger(),
            "Failed to destroy serialized message %s", rcutils_get_error_string().str);
        }
      });
    *bag_message->serialized_data = msg->release_rcl_serialized_message();

    bag_message->topic_name = "/teraranger_evo_thermal/raw_temp_array";
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
        rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
  }

  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}