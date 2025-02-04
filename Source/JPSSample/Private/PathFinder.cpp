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

static const int32 DY[4] = { -1,1,0,0 };
static const int32 DX[4] = { 0,0,-1,1 };

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
	ShowMapTime = 10.0f;
	MapType = EMapType::None;

	PathFindingSimulateCount = 10;

	StartCoord = { -1,-1 };
	EndCoord = { -1,-1 };

	AStarCount = 0;
	JPSCount = 0;
	AStarTime = 0.0;
	JPSTime = 0.0;
}

void APathFinder::BuildMap()
{
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

	if (MapType == EMapType::Navmesh)
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

		//FVector BoxExtent = FVector(Height * IntervalX, Width * IntervalY, HeightLimit) / 2.0f;
		//DrawDebugBox(GetWorld(), GetActorLocation(), BoxExtent, FColor::Green, false, 10.0f);
	}
}

void APathFinder::InitData()
{
	switch (MapType)
	{
	case EMapType::None:
	{
		StartCoord = { 0, 0 };
		EndCoord = { Width - 1, Height - 1 };
	}
		break;
	case EMapType::Random:
	{
		StartCoord = { 0, 0 };
		EndCoord = { Width - 1, Height - 1 };
		float ObstacleRatio = 0.3f;
		for (int32 GridY = 0; GridY < Height; GridY++)
		{
			for (int32 GridX = 0; GridX < Width; GridX++)
			{
				float RandVal = FMath::RandRange(0.f, 1.f);
				if (RandVal < ObstacleRatio)
				{
					if (IsValid(JPSCollision))
					{
						JPSCollision->SetAt(GridY, GridX);
					}

					if (IsValid(AStarCollision))
					{
						AStarCollision->SetNodeAccessibility(GridY, GridX, false);
					}
				}
			}
		}
	}
		break;
	case EMapType::Maze:
	{
		TArray<TArray<uint8>> GridMap = GenerateMaze(Width);
		for (int32 GridY = 0; GridY < Height; GridY++)
		{
			for (int32 GridX = 0; GridX < Width; GridX++)
			{
				bool IsValidCell = GridMap.IsValidIndex(GridX) && GridMap[GridX].IsValidIndex(GridY);
				if (!IsValidCell || GridMap[GridX][GridY] == 1)
				{
					if (IsValid(JPSCollision))
					{
						JPSCollision->SetAt(GridY, GridX);
					}

					if (IsValid(AStarCollision))
					{
						AStarCollision->SetNodeAccessibility(GridY, GridX, false);
					}
				}
			}
		}
	}
		break;
	case EMapType::Room:
	{
		TArray<TArray<uint8>> GridMap = GenerateRoomsMap(Width);
		for (int32 GridY = 0; GridY < GridMap.Num(); GridY++)
		{
			for (int32 GridX = 0; GridX < GridMap[GridY].Num(); GridX++)
			{
				if (GridMap[GridX][GridY] == 1)
				{
					if (IsValid(JPSCollision))
					{
						JPSCollision->SetAt(GridY, GridX);
					}

					if (IsValid(AStarCollision))
					{
						AStarCollision->SetNodeAccessibility(GridY, GridX, false);
					}
				}
			}
		}
	}
		break;
	case EMapType::Block:
	{
		TArray<TArray<uint8>> GridMap = GenerateBlock(Width);
		for (int32 GridY = 0; GridY < GridMap.Num(); GridY++)
		{
			for (int32 GridX = 0; GridX < GridMap[GridY].Num(); GridX++)
			{
				if (GridMap[GridX][GridY] == 1)
				{
					if (IsValid(JPSCollision))
					{
						JPSCollision->SetAt(GridY, GridX);
					}

					if (IsValid(AStarCollision))
					{
						AStarCollision->SetNodeAccessibility(GridY, GridX, false);
					}
				}
			}
		}
	}
		break;
	case EMapType::Navmesh:
	{
		const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
		if (!IsValid(NavSystem))
		{
			UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
			return;
		}

		if (!IsValid(StartActor) || !IsValid(EndActor))
		{
			UE_LOG(LogTemp, Error, TEXT("Not Exist Start or End Point"));
			return;
		}

		FVector CenterLoc = GetActorLocation();
		FVector2D LeftTop = FVector2D(CenterLoc.X + (Height * IntervalX / 2.0f), CenterLoc.Y - (Width * IntervalY / 2.0f));
		for (int32 GridY = 0; GridY < Height; GridY++)
		{
			for (int32 GridX = 0; GridX < Width; GridX++)
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

		StartCoord = LocationToCoord(StartNavLocation.Location);
		EndCoord = LocationToCoord(EndNavLocation.Location);
	}
		break;
	default:
		break;
	}
}

void APathFinder::PathFinding()
{
	if (StartCoord.X == -1 || StartCoord.Y == -1 || EndCoord.X == -1 || EndCoord.Y == -1)
	{
		UE_LOG(LogTemp, Error, TEXT("Invalid Start or End Coord"));
		return;
	}

	bool UseNavmesh = MapType == EMapType::Navmesh;
	if (UseNavmesh)
	{
		const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
		if (!IsValid(NavSystem))
		{
			UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
			return;
		}

		FVector StartLoc = GetNodeLocation(StartCoord.X, StartCoord.Y, true);
		FVector EndLoc = GetNodeLocation(EndCoord.X, EndCoord.Y, true);
		UNavigationPath* NavPath = nullptr;
		{
			FScopedDurationTimeLogger Timer(TEXT("Navmesh"));
			NavPath = NavSystem->FindPathToLocationSynchronously(GetWorld(), StartLoc, EndLoc);
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

					DrawDebugSphere(GetWorld(), NavPath->PathPoints[Route], 50.0f, 2, DrawColor, false, 5.0f);
					if (Route > 0)
					{
						DrawDebugLine(GetWorld(), NavPath->PathPoints[Route], NavPath->PathPoints[Route - 1], FColor::Red, false, 5.0f);
					}
				}
			}
		}
	}

	if (IsValid(AStarCollision))
	{
		FGraph PathFinder(*AStarCollision);
		FSearchNode StartNode(StartCoord);
		FSearchNode EndNode(EndCoord);
		FGridQueryFilter GridQueryFilter;
		TArray<AAStarCollision::FNodeRef> OutPath;

		FDurationTimer AStarTimer(AStarTime);
		PathFinder.FindPath(StartNode, EndNode, GridQueryFilter, OutPath);
		AStarTimer.Stop();
		AStarCount++;
		//{
		//	FScopedDurationTimeLogger Timer(TEXT("GraphAStar"));
		//}
		//UE_LOG(LogTemp, Log, TEXT("AStar Path Length : %d"), OutPath.Num());

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
				FVector Loc = GetNodeLocation(OutPath[Route].X, OutPath[Route].Y, UseNavmesh);

				DrawDebugSphere(GetWorld(), Loc, 50.0f, 2, DrawColor, false, 5.0f);
				if (Route > 0)
				{
					FVector PrevLoc = GetNodeLocation(OutPath[Route - 1].X, OutPath[Route - 1].Y, UseNavmesh);
					DrawDebugLine(GetWorld(), Loc, PrevLoc, FColor::Red, false, 5.0f);
				}
			}
		}
	}

	if (IsValid(JPSCollision))
	{
		TArray<FIntPoint> PathResults;

		FDurationTimer JPSTimer(JPSTime);
		JPSCollision->FindPath(StartCoord, EndCoord, PathResults);
		JPSTimer.Stop();
		JPSCount++;
		//{
		//	FScopedDurationTimeLogger Timer(TEXT("JPS"));
		//}
		//UE_LOG(LogTemp, Log, TEXT("JPS Path Length : %d"), PathResults.Num());

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
				FVector Loc = GetNodeLocation(PathResults[Route].X, PathResults[Route].Y, UseNavmesh);

				DrawDebugSphere(GetWorld(), Loc, 50.0f, 2, DrawColor, false, 5.0f);
				if (Route > 0)
				{
					FVector PrevLoc = GetNodeLocation(PathResults[Route - 1].X, PathResults[Route - 1].Y, UseNavmesh);
					DrawDebugLine(GetWorld(), Loc, PrevLoc, FColor::Red, false, 5.0f);
				}
			}
		}
	}
}

void APathFinder::ShowMap()
{
	for (int32 GridY = 0; GridY < Height; GridY++)
	{
		for (int32 GridX = 0; GridX < Width; GridX++)
		{
			if (IsValid(JPSCollision))
			{
				FVector NodeLoc = GetNodeLocation(GridX, GridY, false);
				DrawDebugSphere(GetWorld(), NodeLoc, 30.0f, 2, JPSCollision->IsCollision(GridX, GridY) ? FColor::Red : FColor::Green, false, ShowMapTime);
			}
		}
	}
}

void APathFinder::PathFindingSimulate()
{
	AStarTime = 0.0;
	JPSTime = 0.0;
	AStarCount = 0;
	JPSCount = 0;
	for (int32 Count = 0; Count < PathFindingSimulateCount; Count++)
	{
		BuildMap();
		InitData();
		PathFinding();
	}

	UE_LOG(LogTemp, Log, TEXT("AStar PathFinding [Test Count = %d] [TestMapSize = %d x %d] [Average Time : %f]"), PathFindingSimulateCount, Width, Height, (float)(AStarTime / AStarCount));
	UE_LOG(LogTemp, Log, TEXT("JPS PathFinding   [Test Count = %d] [TestMapSize = %d x %d] [Average Time : %f]"), PathFindingSimulateCount, Width, Height, (float)(JPSTime / JPSCount));
}

FVector APathFinder::GetNodeLocation(int32 InX, int32 InY, bool InCheckNavmesh)
{
	FVector CenterLoc = GetActorLocation();
	FVector2D LeftTop = FVector2D(CenterLoc.X + (Height * IntervalX / 2.0f), CenterLoc.Y - (Width * IntervalY / 2.0f));
	FVector CenterPos = FVector(LeftTop.X + IntervalX * (-0.5f - InY), LeftTop.Y + IntervalY * (0.5f + InX), CenterLoc.Z);

	if (InCheckNavmesh)
	{
		const UNavigationSystemV1* NavSystem = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
		if (!IsValid(NavSystem))
		{
			UE_LOG(LogTemp, Error, TEXT("Not Exist NavSystem"));
			return FVector(FLT_MAX);
		}

		FNavLocation NavLocation;
		FVector Extent = FVector(0.0f, 0.0f, HeightLimit / 2.0f);
		if (!NavSystem->ProjectPointToNavigation(CenterPos, NavLocation, Extent))
		{
			return FVector(FLT_MAX);
		}

		return NavLocation.Location;
	}

	return CenterPos;
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

bool APathFinder::OverlapsMyBox(const FMyBox& InBoxA, const FMyBox& InBoxB)
{
	if (InBoxA.X + InBoxA.W <= InBoxB.X) return false;
	if (InBoxB.X + InBoxB.W <= InBoxA.X) return false;
	if (InBoxA.Y + InBoxA.H <= InBoxB.Y) return false;
	if (InBoxB.Y + InBoxB.H <= InBoxA.Y) return false;
	return true;
}

TArray<TArray<uint8>> APathFinder::GenerateMaze(int32 InMapSize)
{
	int32 GenerateMapSize = (InMapSize - 1) / 2;
	// 최종 격자 크기는 2N+1
	int32 size = 2 * GenerateMapSize + 1;

	// 전부 벽(1)로 초기화
	TArray<TArray<uint8>> maze;
	maze.SetNum(size);
	for (int32 y = 0; y < size; y++)
	{
		maze[y].Init(1, size);
	}

	// 방문 배열(각 셀은 (y,x) in [0..N-1], 실제 맵 좌표는 (2y+1, 2x+1))
	TArray<TArray<bool>> visited;
	visited.SetNum(GenerateMapSize);
	for (int32 y = 0; y < GenerateMapSize; y++)
	{
		visited[y].Init(false, GenerateMapSize);
	}

	std::srand((unsigned)std::time(nullptr));

	// (0,0) 셀(실제 좌표 (1,1))을 시작점
	visited[0][0] = true;
	// 미로 맵 상에서 (1,1)을 0(통로)
	maze[1][1] = 0;

	// DFS 스택에는 "셀 단위" 좌표를 넣음
	
	TArray<FIntPoint> st;
	st.Add({ 0,0 });

	while (!st.IsEmpty())
	{
		FIntPoint current = st.Last();
		int32 cy = current.X;
		int32 cx = current.Y;
		// cy,cx는 [0..N-1] 범위
		// maze상 실제 좌표는 (2cy+1, 2cx+1)
		int32 cellY = 2 * cy + 1;
		int32 cellX = 2 * cx + 1;

		// 미방문 이웃 찾기
		TArray<int32> candidates;
		for (int32 i = 0; i < 4; i++)
		{
			int32 ny = cy + DY[i];
			int32 nx = cx + DX[i];
			if (ny >= 0 && ny < GenerateMapSize && nx >= 0 && nx < GenerateMapSize)
			{
				if (!visited[ny][nx])
				{
					candidates.Add(i);
				}
			}
		}

		if (!candidates.IsEmpty())
		{
			int32 dir = candidates[rand() % candidates.Num()];
			int32 ny = cy + DY[dir];
			int32 nx = cx + DX[dir];

			// 방문 처리
			visited[ny][nx] = true;
			// 스택에 push
			st.Add({ ny, nx });

			// maze상 실제 좌표
			int32 nextCellY = 2 * ny + 1;
			int32 nextCellX = 2 * nx + 1;
			   
			// 두 셀 사이의 벽 인덱스(둘의 중간 지점)
			int32 wallY = (cellY + nextCellY) / 2;
			int32 wallX = (cellX + nextCellX) / 2;

			// 통로로 변경
			maze[nextCellY][nextCellX] = 0; // 다음 셀
			maze[wallY][wallX] = 0;         // 중간 벽 제거
		}
		else
		{
			st.Pop();
		}
	}

	StartCoord = FIntPoint(1, 1);
	int32 EndPoint = InMapSize % 2 == 0 ? Width - 3 : Width - 2;
	EndCoord = FIntPoint(EndPoint, EndPoint);

	return maze;
}

TArray<TArray<uint8>> APathFinder::GenerateRoomsMap(int32 InMapSize)
{
	// 맵 전체를 1(벽)으로 초기화
	TArray<TArray<uint8>> Map;
	Map.SetNum(Height);
	for (int32 i = 0; i < Height; i++)
	{
		Map[i].Init(1, Width);
	}

	FRandomStream RandomStream(FMath::RandRange(0,9999));

	// (1) 방 생성
	TArray<FMyBox> Rooms;
	int32 Attempts = 0;
	const int32 MaxAttempts = 2000;

	int32 MinRoomCount = InMapSize / 10 > 2 ? InMapSize / 10 : 2;
	int32 MaxRoomCount = InMapSize / 5 > 5 && InMapSize / 5 > MinRoomCount ? InMapSize / 5 : MinRoomCount + 3;
	int32 RoomCount = FMath::RandRange(MinRoomCount, 5);

	int32 MinRoomSize = InMapSize / 10;
	int32 MaxRoomSize = InMapSize / 4;

	StartCoord = { InMapSize ,InMapSize };
	EndCoord = { -1, -1 };
	
	while (Rooms.Num() < RoomCount && Attempts < MaxAttempts)
	{
		Attempts++;
		int32 rw = RandomStream.RandRange(MinRoomSize, MaxRoomSize);
		int32 rh = RandomStream.RandRange(MinRoomSize, MaxRoomSize);
		int32 rx = RandomStream.RandRange(1, InMapSize - rw - 1);
		int32 ry = RandomStream.RandRange(1, InMapSize - rh - 1);

		FMyBox NewRoom{ rx, ry, rw, rh, false};

		// 겹침 체크 (너무 겹치면 제외)
		bool bOverlaps = false;
		for (const FMyBox& R : Rooms)
		{
			if (OverlapsMyBox(NewRoom, R))
			{
				bOverlaps = true;
				break;
			}
		}

		if (!bOverlaps)
		{
			Rooms.Add(NewRoom);
			// 맵에서 방 영역을 0(통로)로 설정
			for (int32 y = ry; y < ry + rh; y++)
			{
				for (int32 x = rx; x < rx + rw; x++)
				{
					Map[y][x] = 0;
				}
			}

			if (rx + ry < StartCoord.X + StartCoord.Y)
			{
				StartCoord.X = rx;
				StartCoord.Y = ry;
			}

			if (rx + rw + ry + rh - 2 > EndCoord.X + EndCoord.Y && StartCoord.X != rx && StartCoord.Y != ry)
			{
				EndCoord.X = rx + rw - 1;
				EndCoord.Y = ry + rh - 1;
			}
		}
	}

	// (2) 방 중심 좌표들 구하기
	TArray<FIntPoint> Centers;
	for (auto& R : Rooms)
	{
		int32 Cx = R.X + R.W / 2;
		int32 Cy = R.Y + R.H / 2;
		Centers.Add(FIntPoint(Cx, Cy));
	}

	// (3) 복도 연결 (간단히: 인접한 두 방 순서대로 연결)
	for (int32 i = 0; i + 1 < Centers.Num(); i++)
	{
		FIntPoint C1 = Centers[i];
		FIntPoint C2 = Centers[i + 1];

		// 가로 방향 먼저 뚫기
		int32 xStep = (C2.X >= C1.X) ? 1 : -1;
		for (int32 x = C1.X; x != C2.X; x += xStep)
		{
			if (x >= 0 && x < InMapSize && C1.Y >= 0 && C1.Y < InMapSize)
			{
				Map[C1.Y][x] = 0;
			}
		}
		// 세로 방향 뚫기
		int32 yStep = (C2.Y >= C1.Y) ? 1 : -1;
		for (int32 y = C1.Y; y != C2.Y; y += yStep)
		{
			if (y >= 0 && y < InMapSize && C2.X >= 0 && C2.X < InMapSize)
			{
				Map[y][C2.X] = 0;
			}
		}
	}

	return Map;
}

TArray<TArray<uint8>> APathFinder::GenerateBlock(int32 InMapSize)
{
	// 맵 전체를 통로(0)로 초기화
	TArray<TArray<uint8>> Map;
	Map.SetNum(InMapSize);
	for (int32 i = 0; i < InMapSize; i++)
	{
		Map[i].Init(0, InMapSize);
	}

	FRandomStream RandomStream(FMath::RandRange(0, 9999));

	int32 Placed = 0;
	int32 Attempts = 0;
	const int32 MaxAttempts = 10000;

	int32 MinBlockCount = InMapSize / 10 > 2 ? InMapSize / 10 : 2;
	int32 MaxBlockCount = InMapSize / 5 > 5 && InMapSize / 5 > MinBlockCount ? InMapSize / 5 : MinBlockCount + 3;
	int32 BlockCount = FMath::RandRange(MinBlockCount, MaxBlockCount);

	int32 MinRoomScale = InMapSize / 10;
	int32 MaxRoomScale = InMapSize / 5;

	while (Placed < BlockCount && Attempts < MaxAttempts)
	{
		Attempts++;
		int32 bw = RandomStream.RandRange(MinRoomScale, MaxRoomScale);
		int32 bh = RandomStream.RandRange(MinRoomScale, MaxRoomScale);
		int32 rx = RandomStream.RandRange(1, InMapSize - bw - 2);
		int32 ry = RandomStream.RandRange(1, InMapSize - bh - 2);

		// 블록(벽) 배치
		for (int32 y = ry; y < ry + bh; y++)
		{
			for (int32 x = rx; x < rx + bw; x++)
			{
				Map[y][x] = 1;  // 벽
			}
		}
		Placed++;
	}

	StartCoord = { 0,0 };
	EndCoord = { InMapSize - 1, InMapSize - 1 };

	return Map;
}

// Called when the game starts or when spawned
void APathFinder::BeginPlay()
{
	Super::BeginPlay();
	
}
