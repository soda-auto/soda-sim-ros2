

#include "Soda/ROS2.h"
#include "GenericPlatform/GenericPlatformProcess.h"
#include "Interfaces/IPluginManager.h"
#include "rcl/logging.h"

static void ROS2OutputHandler(const rcutils_log_location_t* location, int severity, const char* name, rcutils_time_point_value_t timestamp, const char* format, va_list* args)
{
	//rclcpp_logging_output_handler(location, severity, name, timestamp, format, args);
	//rcl_logging_multiple_output_handler( location, severity, name, timestamp, format, args);

	try {
		//std::shared_ptr<std::recursive_mutex> logging_mutex;
		//logging_mutex = get_global_logging_mutex();
		//std::lock_guard<std::recursive_mutex> guard(*logging_mutex);
		rcl_logging_multiple_output_handler(location, severity, name, timestamp, format, args);
	}
	catch (std::exception& ex) {
		RCUTILS_SAFE_FWRITE_TO_STDERR(ex.what());
		RCUTILS_SAFE_FWRITE_TO_STDERR("\n");
	}
	catch (...) {
		RCUTILS_SAFE_FWRITE_TO_STDERR("failed to take global rclcpp logging mutex\n");
	}

	ANSICHAR str[1024];
	int Length = FCStringAnsi::GetVarArgs(str, UE_ARRAY_COUNT(str), format, *args);

	switch (severity)
	{
	case RCUTILS_LOG_SEVERITY_UNSET:
		UE_LOG(LogROS2, Log, TEXT("%s in %s(line %i): %s"), UTF8_TO_TCHAR(location->file_name), UTF8_TO_TCHAR(location->function_name), location->line_number, UTF8_TO_TCHAR(str));
		break;
	case RCUTILS_LOG_SEVERITY_DEBUG:
		UE_LOG(LogROS2, Log, TEXT("%s in %s(line %i): %s"), UTF8_TO_TCHAR(location->file_name), UTF8_TO_TCHAR(location->function_name), location->line_number, UTF8_TO_TCHAR(str));
		break;
	case RCUTILS_LOG_SEVERITY_INFO:
		UE_LOG(LogROS2, Log, TEXT("%s in %s(line %i): %s"), UTF8_TO_TCHAR(location->file_name), UTF8_TO_TCHAR(location->function_name), location->line_number, UTF8_TO_TCHAR(str));
		break;
	case RCUTILS_LOG_SEVERITY_WARN:
		UE_LOG(LogROS2, Warning, TEXT("%s in %s(line %i): %s"), UTF8_TO_TCHAR(location->file_name), UTF8_TO_TCHAR(location->function_name), location->line_number, UTF8_TO_TCHAR(str));
		break;
	case RCUTILS_LOG_SEVERITY_ERROR:
		UE_LOG(LogROS2, Error, TEXT("%s in %s(line %i): %s"), UTF8_TO_TCHAR(location->file_name), UTF8_TO_TCHAR(location->function_name), location->line_number, UTF8_TO_TCHAR(str));
		break;
	case RCUTILS_LOG_SEVERITY_FATAL:
		UE_LOG(LogROS2, Fatal, TEXT("%s in %s(line %i): %s"), UTF8_TO_TCHAR(location->file_name), UTF8_TO_TCHAR(location->function_name), location->line_number, UTF8_TO_TCHAR(str));
		break;
	}
}

FString FSodaROS2Module::AmentPrefixPath = "";

void FSodaROS2Module::StartupModule()
{
	FString ROS2PluginDir;

	if (FPlatformMisc::GetEnvironmentVariable(TEXT("AMENT_PREFIX_PATH")).IsEmpty())
	{
		ROS2PluginDir = FPaths::ConvertRelativePathToFull(IPluginManager::Get().FindPlugin(TEXT("SodaROS2"))->GetBaseDir());
#if PLATFORM_WINDOWS
		FPlatformMisc::SetEnvironmentVar(TEXT("AMENT_PREFIX_PATH"), *FPaths::Combine(ROS2PluginDir, TEXT("/ros2-windows")));
#elif PLATFORM_LINUX
		FPlatformMisc::SetEnvironmentVar(TEXT("AMENT_PREFIX_PATH"), *FPaths::Combine(ROS2PluginDir, TEXT("/ros2-linux")));
#endif
	}

	AmentPrefixPath = FPlatformMisc::GetEnvironmentVariable(TEXT("AMENT_PREFIX_PATH"));

	UE_LOG(LogROS2, Warning, TEXT("FSodaROS2Module::StartupModule(), AMENT_PREFIX_PATH=%s;"), *AmentPrefixPath);

	if (IsRunningCommandlet())
	{
		return;
	}

	FROS2DllDirectoryGuard ROSGuard;

#if PLATFORM_WINDOWS
	TArray<FString> DllPaths =
	{
		FPaths::Combine(ROS2PluginDir, "ros2-ue-wrapper", "Win64", "Bin" , "ros2_ue_wrapper.dll"),
		FPaths::Combine(ROSGuard.GetBinDir(),  "rcl.dll")
	};

	for (auto& It : DllPaths)
	{
		void* LibHandle = FPlatformProcess::GetDllHandle(*It);
		if (LibHandle == nullptr)
		{
			UE_LOG(LogROS2, Fatal, TEXT("FSodaROS2Module::StartupModule(): Failed to load required library %s. Plug-in will not be functional."), *It);
		}
		else
		{
			LoadedDlls.Add(LibHandle);
		}
	}
#endif

	//try
	//{
	ros2_ue_wrapper::Init(false, ROS2OutputHandler);
	//}
	//catch (std::exception& e)
	//{
		//UE_LOG(LogROS2, Error, TEXT("FSodaROS2Module::StartupModule(); rclcpp::init() faild: %s"), UTF8_TO_TCHAR(e.what()));
	//}

	ros2_ue_wrapper::SetLogLevel("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG);
	ros2_ue_wrapper::SetLogLevel("rcl", RCUTILS_LOG_SEVERITY_DEBUG);
	ros2_ue_wrapper::SetLogLevel("rcutils", RCUTILS_LOG_SEVERITY_DEBUG);
	ros2_ue_wrapper::SetLogLevel("rmw", RCUTILS_LOG_SEVERITY_DEBUG);
}

void FSodaROS2Module::ShutdownModule()
{
	if (IsRunningCommandlet())
	{
		return;
	}
	StopExecutor();
	Nodes.Reset();
	
	//try
	//{
	ros2_ue_wrapper::Shutdown();
	//}
	//catch (std::exception& e)
	//{
	//	UE_LOG(LogROS2, Error, TEXT("FSodaROS2Module::ShutdownModule(); rclcpp::shutdown() faild: %s"), UTF8_TO_TCHAR(e.what()));
	//}
}


void FSodaROS2Module::StartExecutor()
{
	StopExecutor();

	if (Nodes.Num())
	{
		Executor = MakeShared<ros2_ue_wrapper::FSingleThreadedExecutor>();

		for (auto& Node : Nodes)
		{
			Executor->AddNode(Node.Value->GetInner());
		}

		Thread = std::thread([&]()
		{
			Executor->Spin();
		});
	}
}

void FSodaROS2Module::StopExecutor()
{
	if (Executor)
	{
		Executor->Cancel();
	}
	if (Thread.joinable()) Thread.join();
	Executor.Reset();
}

TSharedPtr<ros2_ue_wrapper::FNode> FSodaROS2Module::RegistreNode(FName Namespace)
{
	if (auto* FoundNode = Nodes.Find(Namespace))
	{
		return *FoundNode;
	}
	else
	{
		FROS2DllDirectoryGuard Ros2Dir;
		UE_LOG(LogROS2, Log, TEXT("FSodaROS2Module::RegistreNode(); Creating new ROS2 node: %s"), *Namespace.ToString());
		TSharedPtr<ros2_ue_wrapper::FNode> NewNode = MakeShared<ros2_ue_wrapper::FNode>(TCHAR_TO_UTF8(*Namespace.ToString()));
		//std::lock_guard guard(Mutex);
		Nodes.Add(Namespace, NewNode);

		StopExecutor();
		StartExecutor();

		return NewNode;
	}
}

bool FSodaROS2Module::CheckUnregisterNode(FName Namespace)
{
	if (Namespace.IsNone())
	{
		for (auto It = Nodes.CreateIterator(); It; ++It)
		{
			if (It->Value.GetSharedReferenceCount() == 1)
			{
				UE_LOG(LogROS2, Log, TEXT("FSodaROS2Module::CheckUnregisterNode(); Removing ROS2 node: %s"), *It->Key.ToString());
				StopExecutor();
				It.RemoveCurrent();
				StartExecutor();
			}
		}
		return true;
	}
	else
	{
		if (TSharedPtr<ros2_ue_wrapper::FNode>* Node = Nodes.Find(Namespace))
		{
			if (Node->GetSharedReferenceCount() == 1)
			{
				UE_LOG(LogROS2, Log, TEXT("FSodaROS2Module::CheckUnregisterNode(); Removing ROS2 node: %s"), *Namespace.ToString());
				StopExecutor();
				Nodes.Remove(Namespace);
				StartExecutor();
			}
			return true;
		}
	}

	return false;
}

bool FSodaROS2Module::CheckUnregisterNode(TSharedPtr<ros2_ue_wrapper::FNode> Node)
{
	if (auto * Namespace = Nodes.FindKey(Node))
	{
		if (Node.GetSharedReferenceCount() == 1)
		{
			UE_LOG(LogROS2, Log, TEXT("FSodaROS2Module::CheckUnregisterNode(); Removing ROS2 node: %s"), *Namespace->ToString());
			StopExecutor();
			Nodes.Remove(*Namespace);
			StartExecutor();
		}
		return true;
	}
	

	return false;
}

//------------------------------------------------------------------------

FROS2DllDirectoryGuard::FROS2DllDirectoryGuard()
{
	BinDir = FSodaROS2Module::GetAmentPrefixPath() / TEXT("bin");
	FPlatformProcess::PushDllDirectory(*BinDir);
}

FROS2DllDirectoryGuard::~FROS2DllDirectoryGuard()
{
	FPlatformProcess::PopDllDirectory(*BinDir);
}


//------------------------------------------------------------------------
ros2_ue_wrapper::FQoS FQoS::Create() const
{
	ros2_ue_wrapper::FQoS QoS;
	QoS.Depth = Depth;
	QoS.HistoryPolicy = static_cast<ros2_ue_wrapper::EHistoryPolicy>(HistoryPolicy);
	QoS.ReliabilityPolicy = static_cast<ros2_ue_wrapper::EReliabilityPolicy>(ReliabilityPolicy);
	QoS.DurabilityPolicy = static_cast<ros2_ue_wrapper::EDurabilityPolicy>(DurabilityPolicy);
	QoS.LivelinessPolicy = static_cast<ros2_ue_wrapper::ELivelinessPolicy>(LivelinessPolicy);

	//QoS.deadline
	//QoS.lifespan
	//QoS.liveliness_lease_duration
	//QoS.avoid_ros_namespace_conventions

	return QoS;
}

#undef LOCTEXT_NAMESPACE

DEFINE_LOG_CATEGORY(LogROS2);
IMPLEMENT_MODULE(FSodaROS2Module, SodaROS2)