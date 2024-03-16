#include "ros2_ue_wrapper/wrappers.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/illuminance.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/laser_echo.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/region_of_interest.hpp"
#include "sensor_msgs/msg/multi_echo_laser_scan.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

namespace ros2_ue_wrapper
{
	
// -----------------------------------------------------------------------------------------
static rclcpp::QoS GetQoS(const FQoS & InQoS)
{
	rclcpp::QoS QoS(InQoS.HistoryPolicy == EHistoryPolicy::KeepLast ? (rclcpp::QoSInitialization)rclcpp::KeepLast(InQoS.Depth) : (rclcpp::QoSInitialization)rclcpp::KeepAll());
	//QoS.history(static_cast<rclcpp::HistoryPolicy>(HistoryPolicy));
	QoS.reliability(static_cast<rclcpp::ReliabilityPolicy>(InQoS.ReliabilityPolicy));
	QoS.durability(static_cast<rclcpp::DurabilityPolicy>(InQoS.DurabilityPolicy));
	QoS.liveliness(static_cast<rclcpp::LivelinessPolicy>(InQoS.LivelinessPolicy));
	
	return QoS;
}

// -----------------------------------------------------------------------------------------
FNode::FNode(const std::string & Namespace)
{
	Node = std::make_shared<rclcpp::Node>(Namespace);
}

FNode::~FNode()
{
	Node.reset();
}

std::shared_ptr<rclcpp::Node> FNode::GetInner()
{
	return Node;
}

// -----------------------------------------------------------------------------------------
FSingleThreadedExecutor::FSingleThreadedExecutor()
{
	Executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
}

FSingleThreadedExecutor::~FSingleThreadedExecutor()
{
	Executor.reset();
}

std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> FSingleThreadedExecutor::GetInner()
{
	return Executor;
}

void FSingleThreadedExecutor::AddNode(std::shared_ptr<rclcpp::Node> Node, bool notify)
{
	Executor->add_node(Node, notify);
}

void FSingleThreadedExecutor::RemoveNode(std::shared_ptr<rclcpp::Node> Node, bool notify)
{
	Executor->remove_node(Node, notify);	
}

void FSingleThreadedExecutor::Cancel()
{
	Executor->cancel();
}

bool FSingleThreadedExecutor::IsSpinning()
{
	return Executor->is_spinning();
}

void FSingleThreadedExecutor::Spin()
{
	Executor->spin();
}

// -----------------------------------------------------------------------------------------
template<typename MessageT>
class TPublisherImpl
{
public:
	TPublisherImpl() = default;

	~TPublisherImpl()
	{
		Publisher.reset();
	}

	TPublisherImpl(std::shared_ptr<rclcpp::Publisher<MessageT>> Other)
	{
		Publisher = Other;
	}

	TPublisherImpl(const std::shared_ptr<rclcpp::Publisher<MessageT>>& Other)
	{
		Publisher = Other;
	}

	TPublisherImpl(std::shared_ptr<rclcpp::Publisher<MessageT>>&& Other)
	{
		Publisher = std::move(Other);
	}

	TPublisherImpl& operator= (std::shared_ptr<rclcpp::Publisher<MessageT>> Other)
	{
		Publisher = Other;
		return this;
	}

	TPublisherImpl& operator= (const std::shared_ptr<rclcpp::Publisher<MessageT>> & Other)
	{
		Publisher = Other;
		return this;
	}

	TPublisherImpl& operator= (std::shared_ptr<rclcpp::Publisher<MessageT>> && Other)
	{
		Publisher = std::move(Other);
		return this;
	}

	std::shared_ptr < rclcpp::Publisher<MessageT>>& GetInner() { return Publisher; }
	const std::shared_ptr < rclcpp::Publisher<MessageT>>& GetInner() const { return Publisher; }

	std::shared_ptr<rclcpp::Publisher<MessageT>> Publisher;
};

//-----------------------------------------------------------------------------------
template<typename MessageT>
TPublisher<MessageT>::TPublisher(rclcpp::Node & Node, const std::string& Topic, const FQoS& QoS)
{
	Publisher = std::make_shared<TPublisherImpl<MessageT>>();
	Publisher->GetInner() = Node.create_publisher<MessageT>(Topic, GetQoS(QoS));
}

template<typename MessageT>
TPublisher<MessageT>::~TPublisher()
{
	Publisher.reset();
}

template<typename MessageT>
void TPublisher<MessageT>::Publish(const MessageT& msg)
{ 
	Publisher->GetInner()->publish(msg);
}

// -----------------------------------------------------------------------------------------

template<typename MessageT>
class TSubscriptionImpl
{
public:
	TSubscriptionImpl() = default;

	~TSubscriptionImpl()
	{
		Subscription.reset();
	}

	TSubscriptionImpl(std::shared_ptr<rclcpp::Subscription<MessageT>> Other)
	{
		Subscription = Other;
	}

	TSubscriptionImpl(const std::shared_ptr<rclcpp::Subscription<MessageT>>& Other)
	{
		Subscription = Other;
	}

	TSubscriptionImpl(std::shared_ptr<rclcpp::Subscription<MessageT>>&& Other)
	{
		Subscription = std::move(Other);
	}

	TSubscriptionImpl& operator= (std::shared_ptr<rclcpp::Subscription<MessageT>> Other)
	{
		Subscription = Other;
		return this;
	}

	TSubscriptionImpl& operator= (const std::shared_ptr<rclcpp::Subscription<MessageT>>& Other)
	{
		Subscription = Other;
		return this;
	}

	TSubscriptionImpl& operator= (std::shared_ptr<rclcpp::Subscription<MessageT>>&& Other)
	{
		Subscription = std::move(Other);
		return this;
	}

	std::shared_ptr < rclcpp::Subscription<MessageT>>& GetInner() { return Subscription; }
	const std::shared_ptr < rclcpp::Subscription<MessageT>>& GetInner() const { return Subscription; }

	std::shared_ptr<rclcpp::Subscription<MessageT>> Subscription;
};
// -----------------------------------------------------------------------------------------

template<typename MessageT>
TSubscription<MessageT>::TSubscription(rclcpp::Node & Node, const std::string& Topic, const FQoS & QoS)
{
	Subscription = std::make_shared<TSubscriptionImpl<MessageT>>();
	Subscription->GetInner() = Node.create_subscription<MessageT >(Topic, GetQoS(QoS), 
		std::bind(&TSubscription<MessageT>::Callback, this, std::placeholders::_1)
	);
}

template<typename MessageT>
TSubscription<MessageT>::~TSubscription()
{
	Subscription.reset();
}

// -------------------------------------------------------------------------------
void Init(bool bAutoInitializeLogging, rcl_logging_output_handler_t OutputHandler)
{
	const int argc = 1;
	const char* argv[2] = { "", nullptr};

	rclcpp::InitOptions initOptions;
	initOptions.auto_initialize_logging(bAutoInitializeLogging);
	rclcpp::init(argc, argv, initOptions);
	
	if(bAutoInitializeLogging)
	{
		rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context();
		if (rcl_logging_configure_with_output_handler(&context->get_rcl_context()->global_arguments, rcl_init_options_get_allocator(initOptions.get_rcl_init_options()), OutputHandler) != RCL_RET_OK)
		{
			throw std::runtime_error("faild rcl_logging_configure_with_output_handler()");
		}
	}
}

void Shutdown()
{
	rclcpp::shutdown();
}

bool Ok()
{
	return rclcpp::ok();
}

void SetLogLevel(const std::string& LogName, int Level)
{
	rclcpp::get_logger(LogName).set_level(static_cast<rclcpp::Logger::Level>(Level));
}

builtin_interfaces::msg::Time FromNanoseconds(std::uint64_t nanoseconds)
{
	builtin_interfaces::msg::Time msg_time;
	constexpr std::uint64_t kRemainder = 1000000000LL;
	const auto result = std::div(nanoseconds, 1000000000LL);
	if (result.rem >= 0) 
	{
		msg_time.sec = static_cast<std::int32_t>(result.quot);
		msg_time.nanosec = static_cast<std::uint32_t>(result.rem);
	}
	else 
	{
		msg_time.sec = static_cast<std::int32_t>(result.quot - 1);
		msg_time.nanosec = static_cast<std::uint32_t>(kRemainder + result.rem);
	}
	return msg_time;

}

// -------------------------------------------------------------------------------

template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::Image>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::Imu>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::PointCloud>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::PointCloud2>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::LaserScan>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::NavSatFix>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::JointState>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::TimeReference>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::Range>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::Illuminance>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::CameraInfo>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::Temperature>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::LaserEcho>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::RelativeHumidity>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::FluidPressure>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::BatteryState>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::RegionOfInterest>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<sensor_msgs::msg::MultiEchoLaserScan>;

template class ROS2_UE_WRAPPER_PUBLIC TPublisher<nav_msgs::msg::Odometry>;
template class ROS2_UE_WRAPPER_PUBLIC TPublisher<tf2_msgs::msg::TFMessage>;

template class ROS2_UE_WRAPPER_PUBLIC TSubscription<ackermann_msgs::msg::AckermannDriveStamped>;

} // ros2_ue_wrapper