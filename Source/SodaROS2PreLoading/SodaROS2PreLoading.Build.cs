using System;
using System.IO;
using UnrealBuildTool;

public class SodaROS2PreLoading : ModuleRules
{
    public SodaROS2PreLoading(ReadOnlyTargetRules Target) : base(Target)
	{
		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
			}
		);

		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
                "Projects",
            } 
		);
    }
}
