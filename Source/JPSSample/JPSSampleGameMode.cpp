// Copyright Epic Games, Inc. All Rights Reserved.

#include "JPSSampleGameMode.h"
#include "JPSSampleCharacter.h"
#include "UObject/ConstructorHelpers.h"

AJPSSampleGameMode::AJPSSampleGameMode()
{
	// set default pawn class to our Blueprinted character
	static ConstructorHelpers::FClassFinder<APawn> PlayerPawnBPClass(TEXT("/Game/ThirdPerson/Blueprints/BP_ThirdPersonCharacter"));
	if (PlayerPawnBPClass.Class != NULL)
	{
		DefaultPawnClass = PlayerPawnBPClass.Class;
	}
}
