
#include "Interfaces/IPluginManager.h"
#include "HAL/PlatformProcess.h"
#include "Misc/Paths.h"

class FSodaROS2PreLoadingModule : public IModuleInterface
{

	virtual void StartupModule() override
	{
		/*
		FString ROS2PluginDir = FPaths::ConvertRelativePathToFull(IPluginManager::Get().FindPlugin(TEXT("SodaROS2"))->GetBaseDir());

#if PLATFORM_WINDOWS
		FPlatformProcess::AddDllDirectory(*FPaths::Combine(ROS2PluginDir, TEXT("/ros2-windows/bin")));
#elif PLATFORM_LINUX
		FString PreviousLibPath = FPlatformMisc::GetEnvironmentVariable(TEXT("LD_LIBRARY_PATH"));
		if (!PreviousLibPath.IsEmpty())
		{
			PreviousLibPath.Append(TEXT(":"));
		}
		PreviousLibPath.Append(*FPaths::Combine(ROS2PluginDir, TEXT("/ros2-linux/lib")));
		FPlatformMisc::SetEnvironmentVar(TEXT("LD_LIBRARY_PATH"), *PreviousLibPath);
#endif
*/
	}

	virtual void ShutdownModule() override
	{
	}
};

IMPLEMENT_MODULE(FSodaROS2PreLoadingModule, SodaROS2PreLoading)
