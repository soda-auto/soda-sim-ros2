#pragma once

#include "ros2_ue_wrapper/visibility_control.h"
#include <rcl/logging.h>
#include <builtin_interfaces/msg/time.hpp>
#include <cstdio>
#include <functional>

#ifdef _WIN32
#pragma warning( disable: 4251 )
#endif

namespace rclcpp
{
	class Node;
	
	namespace executors
	{
		class SingleThreadedExecutor;
	}
}

namespace ros2_ue_wrapper
{

enum class EHistoryPolicy : std::uint8_t
{
	KeepLast = 1/* RMW_QOS_POLICY_HISTORY_KEEP_LAST */,
	KeepAll  = 2 /* RMW_QOS_POLICY_HISTORY_KEEP_ALL */,
	SystemDefault = 0/* RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT */,
	//Unknown = rclcpp::EHistoryPolicy::Unknown,
};

enum class EReliabilityPolicy : std::uint8_t
{
	BestEffort = 2 /* RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT */,
	Reliable = 1 /* RMW_QOS_POLICY_RELIABILITY_RELIABLE */,
	SystemDefault = 0 /*  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT */,
	//Unknown = rclcpp::ReliabilityPolicy::Unknown,
};

enum class EDurabilityPolicy : std::uint8_t
{
	Volatile = 2 /* RMW_QOS_POLICY_DURABILITY_VOLATILE */,
	TransientLocal = 1/* RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL */,
	SystemDefault = 0/* RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT */,
	//Unknown = rclcpp::DurabilityPolicy::Unknown,
};

enum class ELivelinessPolicy : std::uint8_t
{
	Automatic = 1 /* RMW_QOS_POLICY_LIVELINESS_AUTOMATIC */,
	ManualByTopic = 3 /* RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC */,
	SystemDefault = 0 /* RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT */,
	//Unknown = Automatic::LivelinessPolicy::Unknown,
};

/*
enum class EQoSCompatibility : uint8
{
	Ok = rclcpp::QoSCompatibility::Ok,
	Warning = rclcpp::QoSCompatibility::Warning,
	Error = rclcpp::QoSCompatibility::Error,
};
*/

struct FQoS
{
	int Depth = 5;
	EHistoryPolicy HistoryPolicy = EHistoryPolicy::KeepLast;
	EReliabilityPolicy ReliabilityPolicy = EReliabilityPolicy::Reliable;
	EDurabilityPolicy DurabilityPolicy = EDurabilityPolicy::Volatile;
	ELivelinessPolicy LivelinessPolicy = ELivelinessPolicy::Automatic;
	//double Deadline;
	//double Lifespan;
	//double LivelinessLeaseDuration;
	//bool AvoidRosNamespaceConventions;
};
	

template < typename MessageT > class TPublisherImpl;
template < typename MessageT > class TSubscriptionImpl;

/**
 * FNode
 */
class ROS2_UE_WRAPPER_PUBLIC FNode
{
public:
	FNode(const std::string & Namespace);
	virtual ~FNode();
	
	std::shared_ptr<rclcpp::Node> GetInner();
	
private:
	std::shared_ptr<rclcpp::Node> Node;
};

class ROS2_UE_WRAPPER_PUBLIC FSingleThreadedExecutor
{
public:
	FSingleThreadedExecutor();
	virtual ~FSingleThreadedExecutor();
	
	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> GetInner();
	
	void AddNode(std::shared_ptr<rclcpp::Node> Node, bool notify = true);
	void RemoveNode(std::shared_ptr<rclcpp::Node> Node, bool notify = true);
	void Cancel();
	bool IsSpinning();
	void Spin();
	
private:
	std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> Executor;
};

/**
 * TPublisher
 */
template<typename MessageT>
class ROS2_UE_WRAPPER_PUBLIC TPublisher
{
public:
	TPublisher(rclcpp::Node & Node, const std::string& Topic, const FQoS& QoS);
	virtual ~TPublisher();
	void Publish(const MessageT& msg);

private:
	std::shared_ptr<TPublisherImpl<MessageT> > Publisher;
};

/**
 * TSubscription
 */
template<typename MessageT>
class ROS2_UE_WRAPPER_PUBLIC TSubscription
{
public:
	TSubscription(rclcpp::Node & Node, const std::string& Topic, const FQoS & QoS);
	virtual ~TSubscription();
	
	virtual void Callback(const std::shared_ptr<MessageT> Msg) { }
	
private:
	std::shared_ptr<TSubscriptionImpl<MessageT> > Subscription;
};


ROS2_UE_WRAPPER_PUBLIC void Init(bool bAutoInitializeLogging, rcl_logging_output_handler_t OutputHandler);
ROS2_UE_WRAPPER_PUBLIC void Shutdown();
ROS2_UE_WRAPPER_PUBLIC bool Ok();
ROS2_UE_WRAPPER_PUBLIC builtin_interfaces::msg::Time FromNanoseconds(std::uint64_t nanoseconds);
ROS2_UE_WRAPPER_PUBLIC void SetLogLevel(const std::string& LogName, int Level);

} // ros2_ue_wrapper