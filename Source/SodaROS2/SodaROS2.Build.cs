using System;
using System.IO;
using UnrealBuildTool;

public class SodaROS2 : ModuleRules
{
    public SodaROS2(ReadOnlyTargetRules Target) : base(Target)
	{

        string AmantPrefixPath = null;
        string ROSWrapper = null; 

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            AmantPrefixPath = Path.Combine(PluginDirectory, "ros2-windows");
            ROSWrapper = Path.Combine(PluginDirectory, "ros2-ue-wrapper", "Win64");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            AmantPrefixPath = Path.Combine(PluginDirectory, "ros2-linux");
            ROSWrapper = Path.Combine(PluginDirectory, "ros2-ue-wrapper", "Linux");
        }

        string AmantPrefixIncludePath = Path.Combine(AmantPrefixPath, "include");
        string AmantPrefixLibPath = Path.Combine(AmantPrefixPath, "lib");

        string ROSWrapperIncludePath = Path.Combine(ROSWrapper, "include");
        string ROSWrapperLibPath = Path.Combine(ROSWrapper, "lib");

        PublicIncludePaths.Add(AmantPrefixIncludePath);
        PublicIncludePaths.Add(ROSWrapperIncludePath);
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "std_msgs"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "sensor_msgs"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "geometry_msgs"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "tf2_msgs"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "nav_msgs"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "ackermann_msgs"));

        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "rcl"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "rcl_yaml_param_parser"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "rcutils"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "rmw"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "builtin_interfaces"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "rosidl_runtime_cpp"));
        PublicIncludePaths.Add(Path.Combine(AmantPrefixIncludePath, "rosidl_runtime_c"));

        RuntimeDependencies.Add(Path.Combine(AmantPrefixPath, "..."));
        RuntimeDependencies.Add(Path.Combine(Path.Combine(PluginDirectory, "ros2-ue-wrapper"), "..."));

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ROSWrapperLibPath, "ros2_ue_wrapper.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(AmantPrefixLibPath, "rcl.lib"));

            PublicDelayLoadDLLs.Add("ros2_ue_wrapper.dll");
            PublicDelayLoadDLLs.Add("rcl.dll");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            PublicAdditionalLibraries.Add(Path.Combine(ROSWrapperLibPath, "libros2_ue_wrapper.lib"));
            PublicAdditionalLibraries.Add(Path.Combine(AmantPrefixLibPath, "librcl.a"));

            PublicRuntimeLibraryPaths.Add(AmantPrefixLibPath);
            PrivateRuntimeLibraryPaths.Add(AmantPrefixLibPath);

            PublicRuntimeLibraryPaths.Add(ROSWrapperLibPath);
            PrivateRuntimeLibraryPaths.Add(ROSWrapperLibPath);
        }

        PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
                //"SodaROS2Common",
                "UnrealSoda",
                "RuntimeEditor"
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
                "Projects",
            } 
		);

        //Definitions.Add("_CRT_SECURE_NO_WARNINGS");
        bEnableExceptions = true;
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableUndefinedIdentifierWarnings = false;
		//SetupModulePhysicsSupport(Target);
		//bUsePrecompiled = true;
		bUseUnity = false;
		bUseRTTI = false;
    }
}
