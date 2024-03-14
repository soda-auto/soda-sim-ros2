#pragma once

#include "Modules/ModuleManager.h"
#include <thread>
#include <mutex>

#if !defined (__STDC_VERSION__) 
# define __STDC_VERSION__ 201710L 
#endif
#include "ros2_ue_wrapper/wrappers.h"

#include "ROS2.generated.h"

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
struct FQoS
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


class SODAROS2_API FROS2DllDirectoryGuard
{
public:
	FROS2DllDirectoryGuard();
	~FROS2DllDirectoryGuard();

	const FString& GetBinDir() const { return BinDir; }

protected:
	FString BinDir;
};

namespace ros2
{
	template<typename MessageT>
	class SODAROS2_API TPublisher : public ros2_ue_wrapper::TPublisher<MessageT>
	{
	public:
		TPublisher(TSharedPtr<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS)
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

	template<typename MessageT>
	class SODAROS2_API TSubscription : public ros2_ue_wrapper::TSubscription<MessageT>
	{
	public:
		template<typename TCallback>
		TSubscription(TSharedPtr<ros2_ue_wrapper::FNode> Node, const FString& Topic, const FQoS& QoS, TCallback&& Callback)
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
};


class SODAROS2_API FSodaROS2Module : public IModuleInterface
{
public:

	static inline FSodaROS2Module& Get()
	{
		return FModuleManager::LoadModuleChecked<FSodaROS2Module>("SodaROS2");
	}

	TSharedPtr<ros2_ue_wrapper::FNode> RegistreNode(FName Namespace);
	bool CheckUnregisterNode(FName Namespace = NAME_None);
	bool CheckUnregisterNode(TSharedPtr<ros2_ue_wrapper::FNode> Node);

	template<typename MessageT>
	TSharedPtr<ros2::TPublisher<MessageT>> CreatePublisher(const FString& NodeNamespace, const FString& Topic, struct FQoS& QoS)
	{
		FROS2DllDirectoryGuard Ros2Dir;
		TSharedPtr<ros2_ue_wrapper::FNode> Node;
		try
		{
			Node = RegistreNode(FName(*NodeNamespace));
			return MakeShared<ros2::TPublisher<MessageT>>(Node, Topic, QoS);
		}
		catch (std::exception& e)
		{
			UE_LOG(LogROS2, Error, TEXT("FSodaROS2Module::CreatePublisher(%s): %s"), *Topic, UTF8_TO_TCHAR(e.what()));
		}
		if (!Node) CheckUnregisterNode(FName(*NodeNamespace));
		return {};
	}

	template<typename MessageT, typename TCallback>
	TSharedPtr<ros2::TSubscription<MessageT>> CreateSubscription(const FString& NodeNamespace, const FString& Topic, struct FQoS& QoS, TCallback&& Callback)
	{
		FROS2DllDirectoryGuard Ros2Dir;
		TSharedPtr<ros2_ue_wrapper::FNode> Node;
		try
		{
			Node = RegistreNode(FName(*NodeNamespace));
			return MakeShared<ros2::TSubscription<MessageT>>(Node, Topic, QoS, Callback);
		}
		catch (std::exception& e)
		{
			UE_LOG(LogROS2, Error, TEXT("FSodaROS2Module::CreateSubscription(%s): %s"), *Topic, UTF8_TO_TCHAR(e.what()));
		}
		if (!Node) CheckUnregisterNode(FName(*NodeNamespace));
		return {};
	}



	static FString GetAmentPrefixPath() { return AmentPrefixPath; }

	void StartExecutor();
	void StopExecutor();

protected:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

protected:
#if PLATFORM_WINDOWS
	TArray<void*> LoadedDlls;
#endif
	static FString AmentPrefixPath;
	TMap<FName, TSharedPtr<ros2_ue_wrapper::FNode>> Nodes;
	TSharedPtr<ros2_ue_wrapper::FSingleThreadedExecutor> Executor;
	std::thread Thread;
	std::mutex Mutex;
};
