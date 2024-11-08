// Fill out your copyright notice in the Description page of Project Settings.


#include "MyPlayerController.h"

#include "JPSPath.h"
#include "JPSCollision.h"
#include "Kismet/GameplayStatics.h"
#include "NavigationSystem.h"
#include "DrawDebugHelpers.h"

AMyPlayerController::AMyPlayerController()
{
	JPSPathfinder = CreateDefaultSubobject<UJPSPath>(TEXT("JPSPath"));
}

void AMyPlayerController::BeginPlay()
{
	Super::BeginPlay();

	TSubclassOf<AActor> JPSCollisionClass = AJPSCollision::StaticClass(); 
	AJPSCollision* JPSCollision = Cast<AJPSCollision>(UGameplayStatics::GetActorOfClass(GetWorld(), JPSCollisionClass));
	if (IsValid(JPSCollision) && IsValid(JPSPathfinder))
	{
		JPSPathfinder->SetMap(JPSCollision);
	}
}

bool AMyPlayerController::JPSPathfind()
{
	if (!IsValid(JPSPathfinder))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Exist JPSPathfinder"));
		return false;
	}

	const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!IsValid(NavSystem))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
		return false;
	}

	APawn* MyPawn = GetPawn();
	if (!IsValid(MyPawn))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Connected Pawn"));
		return false;
	}

	TArray<FVector> Results;
	FVector StartLoc = MyPawn->GetActorLocation();
	FNavLocation EndLoc;
	if (!NavSystem->GetRandomPointInNavigableRadius(StartLoc, 1000.0f, EndLoc))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Found NavPath"));
		return false;
	}

	JPSPathfinder->Search(StartLoc, EndLoc.Location, Results);

	for (int32 Route = 0; Route < Results.Num(); Route++)
	{
		FColor DrawColor = FColor::Magenta;
		if (Route == 0)
		{
			DrawColor = FColor::Blue;
		}
		else if (Route == Results.Num() - 1)
		{
			DrawColor = FColor::Purple;
		}
		DrawDebugSphere(GetWorld(), Results[Route], 50.0f, 20, DrawColor, false, 5.0f);
		if (Route > 0)
		{
			DrawDebugLine(GetWorld(), Results[Route], Results[Route - 1], FColor::Red, false, 5.0f);
		}
	}
	return true;
}
