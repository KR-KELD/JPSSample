// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class JPSSample : ModuleRules
{
	public JPSSample(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "NavigationSystem", "AIModule", "ProceduralMeshComponent", "MazeGenerator" });

    }

}
