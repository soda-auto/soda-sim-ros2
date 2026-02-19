#pragma once

#include "Modules/ModuleManager.h"
#include "Soda/SodaApp.h"
#include <thread>
#include <mutex>

#if !defined (__STDC_VERSION__) 
# define __STDC_VERSION__ 201710L 
#endif

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)	// universal-character-name encountered in source
#endif

#include "ros2_ue_wrapper/wrapper.h"

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include "ROS2.generated.h"

#define DEFAULT_ROS2_NODE_NAME TEXT("SodaSim")
#define DEFAULT_ROS2_TOPIC_NAMESPACE TEXT("ego")

#define TOPIC_SENSOR_TEMPLATE TEXT("{namespace}/sensor/{dev_name}")
#define TOPIC_SIG_TEMPLATE TEXT("{namespace}/sig/{dev_name}/{signal_name}")

DECLARE_LOG_CATEGORY_EXTERN(LogROS2, Log, All);

UENUM(BlueprintType)
enum class EHistoryPolicy : uint8
{
	KeepLast = 1/* RMW_QOS_POLICY_HISTORY_KEEP_LAST */,
	KeepAll = 2 /* RMW_QOS_POLICY_HISTORY_KEEP_ALL */,
	SystemDefault = 0/* RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT */,
	//Unknown = rclcpp::EHistoryPolicy::Unknown,
};

UENUM(BlueprintType)
enum class EReliabilityPolicy : uint8
{
	BestEffort = 2 /* RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT */,
	Reliable = 1 /* RMW_QOS_POLICY_RELIABILITY_RELIABLE */,
	SystemDefault = 0 /*  RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT */,
	//Unknown = rclcpp::ReliabilityPolicy::Unknown,
};

UENUM(BlueprintType)
enum class EDurabilityPolicy : uint8
{
	Volatile = 2 /* RMW_QOS_POLICY_DURABILITY_VOLATILE */,
	TransientLocal = 1/* RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL */,
	SystemDefault = 0/* RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT */,
	//Unknown = rclcpp::DurabilityPolicy::Unknown,
};

UENUM(BlueprintType)
enum class ELivelinessPolicy : uint8
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


USTRUCT(BlueprintType)
struct SODAROS2_API FQoS
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	int Depth = 5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	EHistoryPolicy HistoryPolicy = EHistoryPolicy::KeepLast;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	EReliabilityPolicy ReliabilityPolicy = EReliabilityPolicy::Reliable;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	EDurabilityPolicy DurabilityPolicy = EDurabilityPolicy::Volatile;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	ELivelinessPolicy LivelinessPolicy = ELivelinessPolicy::Automatic;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	//double Deadline;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	//double Lifespan;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	//double LivelinessLeaseDuration;

	//UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = QoS, SaveGame, meta = (EditInRuntime))
	//bool AvoidRosNamespaceConventions;

	ros2_ue_wrapper::FQoS Create() const;
};

UENUM(BlueprintType)
enum class ETopicDevNameSbstitution: uint8
{
	/** {dev} will not substitution  */
	Skip,

	/** {dev} will substitution to the vehicle component name */
	ComponentName,

	/** {dev} will substitution to custom value */
	Custom
};

/**
 *  This class offers to format a universal topic name using a template: {namespace}/{type}/{dev_name}/{signal_name}
 *     * {namespace} - default ROS namespace, by default it is taken from the ROS2 node, but can be overrided by "bOverrideNamespace" and "Namespace" properties
 *     * {type} - primary message type. For еxample: vio, vcan, sig, sansor...
 *     * {dev_name} - The name of the device or vehicle component. If DevSubstitution==ETopicKeySubstitution::ComponentName will substitution to the vehicle component name
 *     * {signal_name} - optional field, it can be convenient to use if you need to support multiple messages for one device.
 */
USTRUCT(BlueprintType)
struct SODAROS2_API FROS2TopicSetup
{
	GENERATED_BODY()

	FROS2TopicSetup() = default;
	FROS2TopicSetup(const FString& Topic)
		: Topic(Topic)
	{
	}
	FROS2TopicSetup(const FString& Topic, const FString& SignalName)
		: Topic(Topic)
		, SignalName(SignalName)
	{
	}

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TopicSetup, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString Topic = TEXT("{namespace}/{type}/{dev_name}/{signal_name}");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TopicSetup, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	bool bOverrideNamespace = false;

	/** Substitute {namespace} in the topic. By default, the namespace from the node is used  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TopicSetup, SaveGame, meta = (EditInRuntime, ReactivateComponent, EditCondition = "bOverrideNamespace"))
	FString Namespace = DEFAULT_ROS2_TOPIC_NAMESPACE;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TopicSetup, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	ETopicDevNameSbstitution DevSubstitution = ETopicDevNameSbstitution::ComponentName;

	/** Substitute {dev} in the topic  */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TopicSetup, SaveGame, meta = (EditInRuntime, ReactivateComponent, EditCondition = "DevSubstitution==ETopicDevNameSbstitution::Custom"))
	FString DevName{};

	/** Substitute {signal_name} in the topic */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = TopicSetup, SaveGame, meta = (EditInRuntime, ReactivateComponent))
	FString SignalName{};

	FString GetFormatedTopic(const FString& ComponentName) const;
};


namespace ros2
{
	template<typename MessageT> class TPublisher;
	template<typename MessageT> class TSubscription;
}

class SODAROS2_API FSodaROS2Module : public IModuleInterface
{
public:

	static inline FSodaROS2Module& Get()
	{
		return FModuleManager::LoadModuleChecked<FSodaROS2Module>("SodaROS2");
	}

	TSharedRef<ros2_ue_wrapper::FNode> RegistreNode(FName Namespace);
	bool CheckUnregisterNode(FName Namespace = NAME_None);
	bool CheckUnregisterNode(TSharedRef<ros2_ue_wrapper::FNode> Node);

	static FString GetROSRootPath() { return ROSRootPath; }

	void StartExecutor();
	void StopExecutor();

protected:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

protected:
#if PLATFORM_WINDOWS
	TArray<void*> LoadedDlls;
#endif
	static FString ROSRootPath;

	TMap<FName, TSharedPtr<ros2_ue_wrapper::FNode>> Nodes;
	TSharedPtr<ros2_ue_wrapper::FSingleThreadedExecutor> Executor;
	std::thread Thread;
	std::mutex Mutex;
};


namespace ros2
{
	SODAROS2_API void AddPathToEnv(const FString& Path, const FString& EnvName);

	/**
	 *  TPublisher
	 */
	template<typename MessageT>
	class TPublisher : public ros2_ue_wrapper::TPublisher<MessageT>
	{
	public:
		static TSharedPtr<ros2::TPublisher<MessageT>> Create(TSharedRef<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS)
		{
			try
			{
				return MakeShared<ros2::TPublisher<MessageT>>(Node, Topic, QoS);
			}
			catch (std::exception& e)
			{
				SHOW_NOTIFICATION(Error, 5.0, TEXT("Can't create publisher with topic name: \"%s\". what(): %s"), *Topic, UTF8_TO_TCHAR(e.what()));
			}
			return nullptr;
		}

		static TSharedPtr<ros2::TPublisher<MessageT>> Create(const FString& NodeName, const FString& Topic, const FQoS& QoS)
		{
			return Create(FSodaROS2Module().Get().RegistreNode(FName(*NodeName)), Topic, QoS);
		}

		TPublisher(TSharedRef<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS)
			: ros2_ue_wrapper::TPublisher<MessageT>(*Node->GetInner(), TCHAR_TO_UTF8(*Topic), QoS.Create())
			, Node(Node)
		{}

		~TPublisher()
		{
			Node.Reset();
			FSodaROS2Module::Get().CheckUnregisterNode();
		}
		TSharedPtr<ros2_ue_wrapper::FNode> Node;
	};

	/**
	 *  TSubscription
	 */
	template<typename MessageT>
	class TSubscription : public ros2_ue_wrapper::TSubscription<MessageT>
	{
	public:
		using TCallback = TFunction<void(const std::shared_ptr<MessageT>, const rmw_message_info_t&)>;

		static TSharedPtr<ros2::TSubscription<MessageT>> Create(TSharedRef<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS, TCallback Callback)
		{
			try
			{
				return MakeShared<ros2::TSubscription<MessageT>>(Node, Topic, QoS, Callback);
			}
			catch (std::exception& e)
			{
				SHOW_NOTIFICATION(Error, 5.0, TEXT("Can't create subscription with topic name: \"%s\". what(): %s"), *Topic, UTF8_TO_TCHAR(e.what()));
			}
			return nullptr;
		}

		static TSharedPtr<ros2::TSubscription<MessageT>> Create(const FString& NodeName, const FString& Topic, const FQoS& QoS, TCallback Callback)
		{
			return Create(FSodaROS2Module().Get().RegistreNode(FName(*NodeName)), Topic, QoS, Callback);
		}

		TSubscription(
			TSharedRef<ros2_ue_wrapper::FNode> Node,
			const FString& Topic,
			const FQoS& QoS,
			TFunction<void(const std::shared_ptr<MessageT>, const rmw_message_info_t&)>  Callback)
			: ros2_ue_wrapper::TSubscription<MessageT>(*Node->GetInner(), TCHAR_TO_UTF8(*Topic), QoS.Create(), Callback)
			, Node(Node)
		{}

		~TSubscription()
		{
			Node.Reset();
			FSodaROS2Module::Get().CheckUnregisterNode();
		}

		TSharedPtr<ros2_ue_wrapper::FNode> Node;
	};

	/**
 *  FPubliserSignalBase
 */

	class FPubliserSignalBase
	{
	public:
		FPubliserSignalBase() = default;
		virtual ~FPubliserSignalBase() {};

		virtual void Stop() {};
	};

	/**
	*  FSubscriberSignalBase
	*/

	class FSubscriberSignalBase
	{
	public:
		FSubscriberSignalBase() = default;
		virtual ~FSubscriberSignalBase() {};

		virtual void Stop() {};

	};


	/**
	 *  TPublisherSignal
	 */
	template<typename MessageT>
	class TPublisherSignal : public FPubliserSignalBase
	{
	public:
		using TDataTyp = MessageT::_data_type;

		TPublisherSignal() = default;

		bool Start(TSharedRef<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS)
		{
			Stop();
			Publisher = ros2::TPublisher<MessageT>::Create(Node, Topic, QoS);
			return Publisher.IsValid();
		}

		bool Start(const FString & NodeName, const FString& Topic, const FQoS& QoS)
		{
			return Start(FSodaROS2Module().Get().RegistreNode(FName(*NodeName)), Topic, QoS);
		}

		void Stop()
		{
			Publisher.Reset();
			LastPublishedSignal = TDataTyp{};
			//bSignalIsValid = false;
		}

		void Publish(const TDataTyp & Signal)
		{
			if (Publisher.IsValid())
			{
				MessageT Msg;
				Msg.data = Signal;
				Publisher->Publish(Msg);
			}
			LastPublishedSignal = Signal;
		}

		//bool bIsSiganalValid() const { return bSignalIsValid; }

		TDataTyp GetLastPublishedSignal()
		{
			return LastPublishedSignal;
		}

		TSharedPtr<TPublisher<MessageT>> Publisher;
		TDataTyp LastPublishedSignal;
		//int64 TS_ns;
	};

	/**
	 *  TSubscriptionSignal
	 */
	template<typename MessageT>
	class TSubscriptionSignal : public FSubscriberSignalBase
	{
	public:
		using TDataTyp = MessageT::_data_type;

		TSubscriptionSignal() = default;

		bool Start(TSharedRef<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS)
		{
			Stop();
			Subscription = ros2::TSubscription<MessageT>::Create(Node, Topic, QoS,
				[this](const MessageT::SharedPtr Msg, const rmw_message_info_t& Info)
				{
					Signal = Msg->data;
					bSignalIsValid = true;
					TS_ns = Info.received_timestamp;
				});
			return Subscription.IsValid();
		}

		bool Start(const FString& NodeName, const FString& Topic, const FQoS& QoS)
		{
			return Start(FSodaROS2Module().Get().RegistreNode(FName(*NodeName)), Topic, QoS);
		}

		void Stop()
		{
			Subscription.Reset();
			Signal = TDataTyp{};
			bSignalIsValid = false;
		}

		bool IsSiganalValid() const { return bSignalIsValid; }

		TDataTyp GetSignal()
		{
			return Signal;
		}

		TSharedPtr<TSubscription<MessageT>> Subscription;
		TDataTyp Signal;
		int64 TS_ns;

	private:
		bool bSignalIsValid;
	};


};
