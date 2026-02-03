// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;
using static UnrealBuildTool.ModuleRules;
public class FmiLibAPI : ModuleRules
{
    private string BinariesPath
    {
        get { return Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../Binaries/", Target.Platform.ToString())); }
    }
    public FmiLibAPI(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            string includePath = Path.Combine(ModuleDirectory, "include");
            PublicIncludePaths.AddRange(new string[] { includePath });
            PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "lib", "fmilib_shared.lib"));
            RuntimeDependencies.Add(Path.Combine(BinariesPath, "fmilib_shared.dll"));
            PublicDelayLoadDLLs.Add("fmilib_shared.dll");

            System.Console.WriteLine("Resolved BinariesPath: " + Path.Combine(BinariesPath, "fmilib_shared.dll"));

        }
    }
}


//    private string BinariesPath
//    {
//        get { return Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../Binaries/", Target.Platform.ToString())); }
//    }
//    public VectorXLAPI(ReadOnlyTargetRules Target) : base(Target)
//    {
//    Type = ModuleType.External;

//    if (Target.Platform == UnrealTargetPlatform.Win64)
//    {
//        string includePath = Path.Combine(ModuleDirectory, "Files");
//        PublicIncludePaths.AddRange(new string[] { includePath });
//        PublicAdditionalLibraries.Add(Path.Combine(ModuleDirectory, "Files", "lib", "vxlapi64.lib"));
//        RuntimeDependencies.Add(Path.Combine(BinariesPath, "vxlapi64.dll"));
//        PublicDelayLoadDLLs.Add("vxlapi64.dll");
//    }
//}