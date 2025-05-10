#include <rclcpp/rclcpp.hpp>
#include <smarc_msgs/msg/string_stamped.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <random>

#define SPEED_OF_SOUND 1500.0

using String = std_msgs::msg::String;
using Float32 = std_msgs::msg::Float32;
using StringStamped = smarc_msgs::msg::StringStamped;
using namespace std;

vector<string> split(const string& s, const string& delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

class StringStampedProcessing : public rclcpp::Node
{
public:
    StringStampedProcessing() : Node("sound_distance_calculator")
    {
        string follower_acoustic_topic = this->declare_parameter<string>("follower_acoustic_topic", "");
        string leader1_distance_topic = this->declare_parameter<string>("leader1_distance_topic", "");
        string leader2_distance_topic = this->declare_parameter<string>("leader2_distance_topic", "");
        string leader1_msg_topic = this->declare_parameter<string>("leader1_msg_topic", "");
        string leader2_msg_topic = this->declare_parameter<string>("leader2_msg_topic", "");

        sub_ = this->create_subscription<StringStamped>(
            follower_acoustic_topic, 10, std::bind(&StringStampedProcessing::topicCallback, this, std::placeholders::_1));

        leader1_distance_pub_ = this->create_publisher<Float32>(leader1_distance_topic, 1);
        leader2_distance_pub_ = this->create_publisher<Float32>(leader2_distance_topic, 1);
        leader1_msg_pub_ = this->create_publisher<String>(leader1_msg_topic, 1);
        leader2_msg_pub_ = this->create_publisher<String>(leader2_msg_topic, 1);

        // Initialize random number generator for Gaussian noise
        random_engine_ = std::default_random_engine(std::random_device{}());
        noise_distribution_ = std::normal_distribution<double>(0.0, 0.4); // Mean = 0, Variance = 0.1
    }

private:
    void topicCallback(const StringStamped::SharedPtr msg)
    {
        vector<string> acoustic_msg = split(msg->data, ";");

        rclcpp::Time time_sent = msg->time_sent;
        rclcpp::Time time_received = msg->time_received;

        auto time_diff = time_received - time_sent;
        double time_diff_sec = time_diff.seconds();

        double distance = SPEED_OF_SOUND * time_diff_sec;

        // Add Gaussian noise to the distance
        double noise = noise_distribution_(random_engine_);
        distance += noise;
        
        auto distance_msg = Float32();
        distance_msg.data = distance;

        auto string_msg = String();
        if (acoustic_msg.size() > 1) string_msg.data = acoustic_msg[1];
        
        if (acoustic_msg[0] == "1")
        {
            leader1_distance_pub_->publish(distance_msg);
            if (acoustic_msg.size() > 1) 
            {
                string_msg.data = acoustic_msg[1];
                leader1_msg_pub_->publish(string_msg);
            }
        }
        else if (acoustic_msg[0] == "2")
        {
            leader2_distance_pub_->publish(distance_msg);
            if (acoustic_msg.size() > 1) 
            {
                string_msg.data = acoustic_msg[1];
                leader2_msg_pub_->publish(string_msg);
            }
        }

        // RCLCPP_INFO(this->get_logger(), "Distance sound traveled: %.2f meters", distance);
    }

    rclcpp::Subscription<StringStamped>::SharedPtr sub_;
    rclcpp::Publisher<Float32>::SharedPtr leader1_distance_pub_;
    rclcpp::Publisher<Float32>::SharedPtr leader2_distance_pub_;
    rclcpp::Publisher<String>::SharedPtr leader1_msg_pub_;
    rclcpp::Publisher<String>::SharedPtr leader2_msg_pub_;

    // Random number generator for Gaussian noise
    std::default_random_engine random_engine_;
    std::normal_distribution<double> noise_distribution_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StringStampedProcessing>());
    rclcpp::shutdown();
    return 0;
}