// Fill out your copyright notice in the Description page of Project Settings.


#include "PathFinder.h"

#include "AStarCollision.h"
#include "JPSCollision.h"
#include "Maze.h"
#include "NavigationPath.h"
#include "NavigationSystem.h"
#include "AI/Navigation/NavigationTypes.h"
#include "ProfilingDebugging/ScopedTimers.h"
#include "DrawDebugHelpers.h"

// Sets default values
APathFinder::APathFinder()
{
	Width = 32;
	Height = 32;
	IntervalX = 200.0f;
	IntervalY = 200.0f;
	HeightLimit = 1000.0f;

	AStarCollision = nullptr;
	JPSCollision = nullptr;
	MazeActor = nullptr;
	NavMeshVolumeActor = nullptr;
	StartActor = nullptr;
	EndActor = nullptr;

	JPSDebug = false;
	GraphAStarDebug = false;
	NavmeshDebug = false;
}

void APathFinder::BuildMap()
{
	FVector CurrentLocation = GetActorLocation();
	float HalfWitdh = Height * IntervalX / 2.0f;
	float HalfHeight = Width * IntervalY / 2.0f;
	if (IsValid(MazeActor))
	{
		FVector MazeLocation = CurrentLocation + FVector(-HalfWitdh + 100.0f, -HalfHeight + 100.0f, 0.0f);
		MazeActor->SetActorLocation(MazeLocation);
		MazeActor->MazeSize.X = Width;
		MazeActor->MazeSize.Y = Height;
		MazeActor->UpdateMaze();
	}

	if (IsValid(AStarCollision))
	{
		AStarCollision->BuildMap(Width, Height);
	}

	if (IsValid(JPSCollision))
	{
		JPSCollision->SetWidth(Width);
		JPSCollision->SetHeight(Height);
		JPSCollision->BuildMap();
	}

	if (IsValid(NavMeshVolumeActor))
	{
		NavMeshVolumeActor->SetActorLocation(CurrentLocation);
		NavMeshVolumeActor->SetActorScale3D(FVector(Width + 2.0f, Height + 2.0f, 2.0f));
	}

	if (IsValid(StartActor))
	{
		FVector StartLocation = CurrentLocation + FVector(-HalfWitdh + 100.0f, -HalfHeight + 100.0f, 0.0f);
		StartActor->SetActorLocation(StartLocation);
	}

	if (IsValid(EndActor))
	{
		FVector EndLocation = CurrentLocation + FVector(HalfWitdh - 300.0f, HalfHeight - 300.0f, 0.0f);
		EndActor->SetActorLocation(EndLocation);
	}

	FVector BoxExtent = FVector(Height * IntervalX, Width * IntervalY, HeightLimit) / 2.0f;
	DrawDebugBox(GetWorld(), GetActorLocation(), BoxExtent, FColor::Green, false, 10.0f);
}

void APathFinder::CalcCollision()
{
	const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!IsValid(NavSystem))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
		return;
	}

	FVector CenterLoc = GetActorLocation();
	FVector2D LeftTop = FVector2D(CenterLoc.X + (Height * IntervalX / 2.0f), CenterLoc.Y - (Width * IntervalY / 2.0f));
	for (int32 GridY = 0; GridY < Height; ++GridY)
	{
		for (int32 GridX = 0; GridX < Width; ++GridX)
		{
			FVector CellLoc = FVector(LeftTop.X + IntervalX * (-0.5f - GridY), LeftTop.Y + IntervalY * (0.5f + GridX), CenterLoc.Z + HeightLimit / 2.0f);
			FNavLocation NavLocation;
			FVector Extent = FVector(0.0f, 0.0f, HeightLimit / 2.0f);
			// 내비메시가 존재하지 않는 구역은 닫힘처리 
			if (!NavSystem->ProjectPointToNavigation(CellLoc, NavLocation, Extent))
			{

				if (IsValid(JPSCollision))
				{
					JPSCollision->SetAt(GridX, GridY);
				}

				if (IsValid(AStarCollision))
				{
					AStarCollision->SetNodeAccessibility(GridX, GridY, false);
				}
			}
		}
	}
}

void APathFinder::PathFinding()
{
	const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!IsValid(NavSystem))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
		return;
	}

	if (!IsValid(StartActor) || !IsValid(EndActor))
	{
		return;
	}

	FNavLocation StartNavLocation;
	if (!NavSystem->GetRandomPointInNavigableRadius(StartActor->GetActorLocation(), IntervalX / 2.0f, StartNavLocation))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Found Start NavLocation"));
		return;
	}

	FNavLocation EndNavLocation;
	if (!NavSystem->GetRandomPointInNavigableRadius(EndActor->GetActorLocation(), IntervalX / 2.0f, EndNavLocation))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Found End NavLocation"));
		return;
	}

	FIntPoint StartCoord = LocationToCoord(StartNavLocation.Location);
	FIntPoint EndCoord = LocationToCoord(EndNavLocation.Location);

	if (IsValid(AStarCollision))
	{
		FGraph PathFinder(*AStarCollision);
		FSearchNode StartNode(StartCoord);
		FSearchNode EndNode(EndCoord);
		FGridQueryFilter GridQueryFilter;
		TArray<AAStarCollision::FNodeRef> OutPath;

		{
			FScopedDurationTimeLogger Timer(TEXT("GraphAStar"));
			PathFinder.FindPath(StartNode, EndNode, GridQueryFilter, OutPath);
		}

		if (GraphAStarDebug)
		{
			for (int32 Route = 0; Route < OutPath.Num(); Route++)
			{
				FColor DrawColor = FColor::Magenta;
				if (Route == 0)
				{
					DrawColor = FColor::Blue;
				}
				else if (Route == OutPath.Num() - 1)
				{
					DrawColor = FColor::Purple;
				}
				FVector Loc = GetNodeLocation(OutPath[Route].X, OutPath[Route].Y);

				DrawDebugSphere(GetWorld(), Loc, 50.0f, 20, DrawColor, false, 5.0f);
				if (Route > 0)
				{
					FVector PrevLoc = GetNodeLocation(OutPath[Route - 1].X, OutPath[Route - 1].Y);
					DrawDebugLine(GetWorld(), Loc, PrevLoc, FColor::Red, false, 5.0f);
				}
			}
		}
	}

	if (IsValid(JPSCollision))
	{
		TArray<FIntPoint> PathResults;
		{
			FScopedDurationTimeLogger Timer(TEXT("JPS"));
			JPSCollision->FindPath(StartCoord, EndCoord, PathResults);
		}

		if (JPSDebug)
		{
			for (int32 Route = 0; Route < PathResults.Num(); Route++)
			{
				FColor DrawColor = FColor::Magenta;
				if (Route == 0)
				{
					DrawColor = FColor::Blue;
				}
				else if (Route == PathResults.Num() - 1)
				{
					DrawColor = FColor::Purple;
				}
				FVector Loc = GetNodeLocation(PathResults[Route].X, PathResults[Route].Y);

				DrawDebugSphere(GetWorld(), Loc, 50.0f, 20, DrawColor, false, 5.0f);
				if (Route > 0)
				{
					FVector PrevLoc = GetNodeLocation(PathResults[Route - 1].X, PathResults[Route - 1].Y);
					DrawDebugLine(GetWorld(), Loc, PrevLoc, FColor::Red, false, 5.0f);
				}
			}
		}
	}

	UNavigationSystemV1* NavSys = Cast<UNavigationSystemV1>(GetWorld()->GetNavigationSystem());
	if (NavSys)
	{
		UNavigationPath* NavPath = nullptr;
		{
			FScopedDurationTimeLogger Timer(TEXT("Navmesh"));
			NavPath = NavSys->FindPathToLocationSynchronously(GetWorld(), StartNavLocation.Location, EndNavLocation.Location);
		}

		if (NavmeshDebug)
		{
			if (NavPath && NavPath->PathPoints.Num() > 1)
			{
				for (int32 Route = 0; Route < NavPath->PathPoints.Num(); Route++)
				{
					FColor DrawColor = FColor::Magenta;
					if (Route == 0)
					{
						DrawColor = FColor::Blue;
					}
					else if (Route == NavPath->PathPoints.Num() - 1)
					{
						DrawColor = FColor::Purple;
					}

					DrawDebugSphere(GetWorld(), NavPath->PathPoints[Route], 50.0f, 20, DrawColor, false, 5.0f);
					if (Route > 0)
					{
						DrawDebugLine(GetWorld(), NavPath->PathPoints[Route], NavPath->PathPoints[Route - 1], FColor::Red, false, 5.0f);
					}
				}
			}
		}
	}
}

FVector APathFinder::GetNodeLocation(int32 InX, int32 InY)
{
	const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
	if (!IsValid(NavSystem))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
		return FVector(FLT_MAX);
	}
	FVector CenterLoc = GetActorLocation();
	FVector2D LeftTop = FVector2D(CenterLoc.X + (Height * IntervalX / 2.0f), CenterLoc.Y - (Width * IntervalY / 2.0f));

	FVector CenterPos = FVector(LeftTop.X + IntervalX * (-0.5f - InY), LeftTop.Y + IntervalY * (0.5f + InX), CenterLoc.Z);
	FNavLocation NavLocation;
	FVector Extent = FVector(0.0f, 0.0f, HeightLimit / 2.0f);
	if (!NavSystem->ProjectPointToNavigation(CenterPos, NavLocation, Extent))
	{
		return FVector(FLT_MAX);
	}

	return NavLocation.Location;
}

FIntPoint APathFinder::LocationToCoord(FVector InLocation)
{
	FIntPoint Result = { -1,-1 };
	FVector CenterLoc = GetActorLocation();

	FVector2D LeftTop = FVector2D(CenterLoc.X + (Height * IntervalX / 2.0f), CenterLoc.Y - (Width * IntervalY / 2.0f));
	FVector2D RightBottom = FVector2D(CenterLoc.X - (Height * IntervalX / 2.0f), CenterLoc.Y + (Width * IntervalY / 2.0f));

	if (InLocation.Y < LeftTop.Y || InLocation.Y > RightBottom.Y ||
		InLocation.X > LeftTop.X || InLocation.X < RightBottom.X ||
		InLocation.Z > CenterLoc.Z + HeightLimit / 2.0f ||
		InLocation.Z < CenterLoc.Z - HeightLimit / 2.0f)
	{
		return Result;
	}

	Result.X = (InLocation.Y - LeftTop.Y) / IntervalY;
	Result.Y = -(InLocation.X - LeftTop.X) / IntervalX;

	return Result;
}

// Called when the game starts or when spawned
void APathFinder::BeginPlay()
{
	Super::BeginPlay();
	
}
