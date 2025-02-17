using System;
using System.IO;
using UnrealBuildTool;

public class SodaROS2 : ModuleRules
{
    public SodaROS2(ReadOnlyTargetRules Target) : base(Target)
	{

        PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
                //"SodaROS2Common",
                "UnrealSoda",
                "RuntimeEditor",
				"ROSHumble"
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
                "Projects",
                "Slate",
                "SlateCore"
            } 
		);

        //Definitions.Add("_CRT_SECURE_NO_WARNINGS");
        bEnableExceptions = true;
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
		bEnableUndefinedIdentifierWarnings = false;
		//SetupModulePhysicsSupport(Target);
		//bUsePrecompiled = true;
		bUseUnity = false;
    }
}
