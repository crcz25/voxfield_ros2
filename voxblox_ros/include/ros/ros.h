#ifndef VOXBLOX_ROS_COMPAT_ROS_H_
#define VOXBLOX_ROS_COMPAT_ROS_H_

#include <chrono>
#include <cstdio>
#include <ostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ros {

inline rclcpp::Node::SharedPtr& globalNode() {
  static rclcpp::Node::SharedPtr node;
  return node;
}

inline rclcpp::Logger get_logger() {
  return globalNode() ? globalNode()->get_logger() : rclcpp::get_logger("voxblox_ros");
}

inline rclcpp::Clock::SharedPtr get_clock() {
  static rclcpp::Clock::SharedPtr fallback =
      std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  return globalNode() ? globalNode()->get_clock() : fallback;
}

class Duration {
 public:
  Duration() : seconds_(0.0) {}
  explicit Duration(double seconds) : seconds_(seconds) {}

  void fromSec(double seconds) {
    seconds_ = seconds;
  }

  double toSec() const {
    return seconds_;
  }

  int64_t toNSec() const {
    return static_cast<int64_t>(seconds_ * 1.0e9);
  }

  std::chrono::nanoseconds toChrono() const {
    return std::chrono::nanoseconds(toNSec());
  }

  operator builtin_interfaces::msg::Duration() const {
    builtin_interfaces::msg::Duration msg;
    const int64_t nanoseconds = toNSec();
    msg.sec = static_cast<int32_t>(nanoseconds / 1000000000ll);
    msg.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ll);
    return msg;
  }

 private:
  double seconds_;
};

class Time {
 public:
  Time() : time_(0, 0, RCL_ROS_TIME) {}
  Time(const builtin_interfaces::msg::Time& stamp)
      : time_(stamp, RCL_ROS_TIME) {}
  explicit Time(const rclcpp::Time& time) : time_(time) {}

  Time& operator=(const builtin_interfaces::msg::Time& stamp) {
    time_ = rclcpp::Time(stamp, RCL_ROS_TIME);
    return *this;
  }

  int64_t toNSec() const {
    return time_.nanoseconds();
  }

  double toSec() const {
    return time_.seconds();
  }

  builtin_interfaces::msg::Time toMsg() const {
    return time_;
  }

  operator builtin_interfaces::msg::Time() const {
    return toMsg();
  }

  const rclcpp::Time& rclcppTime() const {
    return time_;
  }

  static Time now() {
    return Time(get_clock()->now());
  }

 private:
  rclcpp::Time time_;
};

inline Duration operator-(const Time& lhs, const Time& rhs) {
  return Duration((lhs.toNSec() - rhs.toNSec()) / 1.0e9);
}

inline Duration operator-(const builtin_interfaces::msg::Time& lhs,
                          const Time& rhs) {
  return Time(lhs) - rhs;
}

inline Duration operator-(const Time& lhs,
                          const builtin_interfaces::msg::Time& rhs) {
  return lhs - Time(rhs);
}

inline Duration operator-(const builtin_interfaces::msg::Time& lhs,
                          const builtin_interfaces::msg::Time& rhs) {
  return Time(lhs) - Time(rhs);
}

inline bool operator>(const builtin_interfaces::msg::Time& lhs,
                      const Time& rhs) {
  return Time(lhs).toNSec() > rhs.toNSec();
}

inline bool operator>(const builtin_interfaces::msg::Time& lhs,
                      const builtin_interfaces::msg::Time& rhs) {
  return Time(lhs).toNSec() > Time(rhs).toNSec();
}

inline bool operator>(const Duration& lhs, const Duration& rhs) {
  return lhs.toNSec() > rhs.toNSec();
}

inline std::ostream& operator<<(std::ostream& stream, const Time& time) {
  stream << time.toSec();
  return stream;
}

inline std::ostream& operator<<(
    std::ostream& stream, const builtin_interfaces::msg::Time& time) {
  stream << Time(time).toSec();
  return stream;
}

class WallTime {
 public:
  WallTime() : time_(std::chrono::steady_clock::now()) {}
  static WallTime now() {
    return WallTime();
  }
  Duration operator-(const WallTime& other) const {
    return Duration(std::chrono::duration<double>(time_ - other.time_).count());
  }

 private:
  std::chrono::steady_clock::time_point time_;
};

struct TimerEvent {};

class Publisher {
 public:
  Publisher() = default;

  template <typename MessageT>
  explicit Publisher(
      std::shared_ptr<rclcpp::Publisher<MessageT>> publisher)
      : holder_(std::make_shared<TypedHolder<MessageT>>(publisher)) {}

  template <typename PointT>
  explicit Publisher(
      std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> publisher,
      pcl::PointCloud<PointT>*)
      : holder_(std::make_shared<PclHolder<PointT>>(publisher)) {}

  template <typename MessageT>
  void publish(const MessageT& message) const {
    holder_->publishAny(&message);
  }

  int getNumSubscribers() const {
    return holder_ ? static_cast<int>(holder_->countSubscribers()) : 0;
  }

 private:
  struct HolderBase {
    virtual ~HolderBase() = default;
    virtual void publishAny(const void* message) = 0;
    virtual size_t countSubscribers() const = 0;
  };

  template <typename MessageT>
  struct TypedHolder : HolderBase {
    explicit TypedHolder(typename rclcpp::Publisher<MessageT>::SharedPtr pub)
        : pub_(pub) {}
    void publishAny(const void* message) override {
      pub_->publish(*static_cast<const MessageT*>(message));
    }
    size_t countSubscribers() const override {
      return pub_->get_subscription_count();
    }
    typename rclcpp::Publisher<MessageT>::SharedPtr pub_;
  };

  template <typename PointT>
  struct PclHolder : HolderBase {
    explicit PclHolder(
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub)
        : pub_(pub) {}
    void publishAny(const void* message) override {
      const auto& pcl_cloud =
          *static_cast<const pcl::PointCloud<PointT>*>(message);
      sensor_msgs::msg::PointCloud2 ros_cloud;
      pcl::toROSMsg(pcl_cloud, ros_cloud);
      ros_cloud.header.frame_id = pcl_cloud.header.frame_id;
      pub_->publish(ros_cloud);
    }
    size_t countSubscribers() const override {
      return pub_->get_subscription_count();
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  };

  std::shared_ptr<HolderBase> holder_;
};

using Subscriber = rclcpp::SubscriptionBase::SharedPtr;
using ServiceServer = rclcpp::ServiceBase::SharedPtr;
using Timer = rclcpp::TimerBase::SharedPtr;

template <typename T>
struct is_pcl_cloud : std::false_type {};

template <typename PointT>
struct is_pcl_cloud<pcl::PointCloud<PointT>> : std::true_type {
  using PointType = PointT;
};

class NodeHandle {
 public:
  NodeHandle() : private_(false) {}
  explicit NodeHandle(const std::string& ns) : private_(ns == "~") {}

  template <typename T>
  void param(const std::string& name, T& value, const T& default_value) const {
    auto node = globalNode();
    if constexpr (std::is_floating_point_v<T>) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<double>(name, static_cast<double>(default_value));
      }
      value = static_cast<T>(node->get_parameter(name).as_double());
    } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
      }
      value = static_cast<T>(node->get_parameter(name).as_int());
    } else {
      if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
      }
      value = node->get_parameter(name).get_value<T>();
    }
  }

  template <typename T>
  bool getParam(const std::string& name, T& value) const {
    auto node = globalNode();
    if (!node->has_parameter(name)) {
      return false;
    }
    return node->get_parameter(name, value);
  }

  template <typename MessageT>
  Publisher advertise(
      const std::string& topic, size_t queue_size, bool latched = false) const {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));
    if (latched) {
      qos.transient_local();
    }
    if constexpr (is_pcl_cloud<MessageT>::value) {
      auto publisher = globalNode()->create_publisher<sensor_msgs::msg::PointCloud2>(
          resolveTopic(topic), qos);
      return Publisher(
          publisher,
          static_cast<pcl::PointCloud<typename is_pcl_cloud<MessageT>::PointType>*>(
              nullptr));
    } else {
      return Publisher(globalNode()->create_publisher<MessageT>(
          resolveTopic(topic), qos));
    }
  }

  template <typename MessageT, typename ClassT>
  Subscriber subscribe(
      const std::string& topic, size_t queue_size,
      void (ClassT::*callback)(const std::shared_ptr<MessageT>&),
      ClassT* object) const {
    return globalNode()->create_subscription<MessageT>(
        resolveTopic(topic), rclcpp::QoS(rclcpp::KeepLast(queue_size)),
        [object, callback](std::shared_ptr<MessageT> msg) {
          (object->*callback)(msg);
        });
  }

  template <typename MessageT, typename ClassT>
  Subscriber subscribe(
      const std::string& topic, size_t queue_size,
      void (ClassT::*callback)(const MessageT&), ClassT* object) const {
    return globalNode()->create_subscription<MessageT>(
        resolveTopic(topic), rclcpp::QoS(rclcpp::KeepLast(queue_size)),
        [object, callback](typename MessageT::SharedPtr msg) {
          (object->*callback)(*msg);
        });
  }

  template <typename ServiceT, typename ClassT>
  ServiceServer advertiseService(
      const std::string& name,
      bool (ClassT::*callback)(typename ServiceT::Request&,
                               typename ServiceT::Response&),
      ClassT* object) const {
    return globalNode()->create_service<ServiceT>(
        resolveTopic(name),
        [object, callback](
            const std::shared_ptr<typename ServiceT::Request> request,
            std::shared_ptr<typename ServiceT::Response> response) {
          (void)(object->*callback)(*request, *response);
        });
  }

  template <typename ClassT>
  Timer createTimer(
      const Duration& duration, void (ClassT::*callback)(const TimerEvent&),
      ClassT* object) const {
    return globalNode()->create_wall_timer(
        duration.toChrono(), [object, callback]() {
          TimerEvent event;
          (object->*callback)(event);
        });
  }

 private:
  std::string resolveTopic(const std::string& topic) const {
    if (!private_ || topic.empty() || topic[0] == '/' || topic[0] == '~') {
      return topic;
    }
    return "~/" + topic;
  }

  bool private_;
};

inline void init(int& argc, char** argv, const std::string& node_name) {
  rclcpp::init(argc, argv);

  static std::vector<std::string> non_ros_args;
  static std::vector<char*> non_ros_argv;
  non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  non_ros_argv.clear();
  non_ros_argv.reserve(non_ros_args.size());
  for (std::string& arg : non_ros_args) {
    non_ros_argv.push_back(arg.data());
  }
  argc = static_cast<int>(non_ros_argv.size());
  for (int i = 0; i < argc; ++i) {
    argv[i] = non_ros_argv[i];
  }

  globalNode() = std::make_shared<rclcpp::Node>(
      node_name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
}

inline void spin() {
  rclcpp::spin(globalNode());
}

inline void spinOnce() {
  rclcpp::spin_some(globalNode());
}

inline void shutdown() {
  rclcpp::shutdown();
}

}  // namespace ros

namespace builtin_interfaces {
namespace msg {

template <typename Allocator>
inline std::ostream& operator<<(
    std::ostream& stream, const Time_<Allocator>& time) {
  stream << (static_cast<double>(time.sec) +
             static_cast<double>(time.nanosec) * 1.0e-9);
  return stream;
}

}  // namespace msg
}  // namespace builtin_interfaces

#define ROS_INFO(...) RCLCPP_INFO(::ros::get_logger(), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(::ros::get_logger(), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(::ros::get_logger(), __VA_ARGS__)
#define ROS_FATAL(...) RCLCPP_FATAL(::ros::get_logger(), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(::ros::get_logger(), __VA_ARGS__)

#define ROS_INFO_STREAM(expr) \
  do { std::ostringstream _ros_stream; _ros_stream << expr; RCLCPP_INFO(::ros::get_logger(), "%s", _ros_stream.str().c_str()); } while (0)
#define ROS_WARN_STREAM(expr) \
  do { std::ostringstream _ros_stream; _ros_stream << expr; RCLCPP_WARN(::ros::get_logger(), "%s", _ros_stream.str().c_str()); } while (0)
#define ROS_ERROR_STREAM(expr) \
  do { std::ostringstream _ros_stream; _ros_stream << expr; RCLCPP_ERROR(::ros::get_logger(), "%s", _ros_stream.str().c_str()); } while (0)
#define ROS_FATAL_STREAM(expr) \
  do { std::ostringstream _ros_stream; _ros_stream << expr; RCLCPP_FATAL(::ros::get_logger(), "%s", _ros_stream.str().c_str()); } while (0)
#define ROS_DEBUG_STREAM(expr) \
  do { std::ostringstream _ros_stream; _ros_stream << expr; RCLCPP_DEBUG(::ros::get_logger(), "%s", _ros_stream.str().c_str()); } while (0)

#define ROS_INFO_ONCE(...) \
  do { static bool _ros_once = false; if (!_ros_once) { _ros_once = true; ROS_INFO(__VA_ARGS__); } } while (0)
#define ROS_WARN_THROTTLE(period, ...) ROS_WARN(__VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) ROS_ERROR(__VA_ARGS__)
#define ROS_WARN_STREAM_THROTTLE(period, expr) ROS_WARN_STREAM(expr)
#define ROS_ERROR_STREAM_THROTTLE(period, expr) ROS_ERROR_STREAM(expr)

#endif  // VOXBLOX_ROS_COMPAT_ROS_H_
