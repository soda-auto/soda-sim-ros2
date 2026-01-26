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
                "UnrealSoda",
                "RuntimeEditor",
                "ROSJazzy" // "ROSHumble" or "ROSJazzy"
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
        UndefinedIdentifierWarningLevel = WarningLevel.Off;
        //SetupModulePhysicsSupport(Target);
        //bUsePrecompiled = true;
        bUseUnity = false;
    }
}
