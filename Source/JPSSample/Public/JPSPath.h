// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "JPSCore.h"
#include "TDBitArray.h"
#include "JPSCollision.h"

#include "JPSPath.generated.h"
/**
 * 
 */
UCLASS()
class UJPSPath : public UObject
{
	GENERATED_BODY()
public:

	UJPSPath();

	void SetMap(AJPSCollision* InFieldCollision);
	void DestroyMap();
	bool Search(FVector InStartLoc, FVector InEndLoc, TArray<FVector>& OutResultPos);

private:

	inline bool IsPassable(const JPSCoord& InCoord)
	{
		return !FieldCollision->IsCollision(InCoord.X, InCoord.Y);
	}

	inline int32 DirIsDiagonal(const int32 InDir)
	{
		// 대각선인지 판단
		return (InDir % 2) != 0;
	}

	inline int32 Implies(const int32 InA, const int32 InB)
	{
		// a와 b의 값이 둘 다 존재하거나 둘 다 존재하지 않으면 참
		return InA ? InB : 1;
	}

	inline int32 AddDirectionToSet(const int32 InDirs, const int32 InDir)
	{
		// 방향 추가
		return InDirs | 1 << InDir;
	}

	TPair<int32, int32> GetNorthEndPointReOpenBB(int32 InX, int32 InY);
	TPair<int32, int32> GetSouthEndPointReOpenBB(int32 InX, int32 InY);
	TPair<int32, int32> GetEastEndPointReOpenBB(int32 InX, int32 InY);
	TPair<int32, int32> GetWestEndPointReOpenBB(int32 InX, int32 InY);

	JPSCoord NextCoordinate(const JPSCoord& InCoord, const int32 InDir);
	int32 GetCoordinateDir(const JPSCoord& InSCoord, const JPSCoord& InDirCoord);
	int32 GetForcedNeighbours(const JPSCoord& InCoord, const int32 InDir);
	int32 GetNaturalNeighbours(const int32 InDir);

	bool GetJumpPoint(JPSCoord InSCoord, const char direction, JPSCoord& OutJumpPoint);
	JPSCoord Jump(const JPSCoord& InCoord, const char InDir);

public:
	bool PullingString(TArray<JPSCoord>& InResultNodes);
	bool IsStraightPassable(int32 InFromX, int32 InFromY, int32 InToX, int32 InToY);

private:
	// 방향
	// 북(0), 북동(1), 동(2), 남동(3), 남(4), 남서(5), 서(6), 북서(7) , 시작점(8방)
	const int32	NODIRECTION = 8;

	// 열 노드
	UPROPERTY()
	UJPSHeap* OpenList;

	// 닫힌 노드
	TDBitArray<int64> ClosedList;

	JPSCoord EndPos;

	TWeakObjectPtr<AJPSCollision> FieldCollision;
	int32 GridWidth = 0;
	int32 GridHeight = 0;
};
