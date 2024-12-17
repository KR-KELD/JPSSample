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
	bool Search(FIntPoint InStartCoord, FIntPoint InEndCoord, TArray<FIntPoint>& OutResultCoord);

private:

	inline bool IsPassable(const JPSCoord& InCoord)
	{
		if (FieldCollision.IsValid())
		{
			return !FieldCollision->IsOutBound(InCoord.X, InCoord.Y) && !FieldCollision->IsCollision(InCoord.X, InCoord.Y);
		}
		return false;
	}

	inline int32 DirIsDiagonal(const int32 InDir)
	{
		// �밢������ �Ǵ�
		return (InDir % 2) != 0;
	}

	inline int32 Implies(const int32 InA, const int32 InB)
	{
		// a�� b�� ���� �� �� �����ϰų� �� �� �������� ������ ��
		return InA ? InB : 1;
	}

	inline int32 AddDirectionToSet(const int32 InDirs, const int32 InDir)
	{
		// ���� �߰�
		return InDirs | 1 << InDir;
	}

	FIntPoint GetNorthEndPointReOpenBB(int32 InX, int32 InY);
	FIntPoint GetSouthEndPointReOpenBB(int32 InX, int32 InY);
	FIntPoint GetEastEndPointReOpenBB(int32 InX, int32 InY);
	FIntPoint GetWestEndPointReOpenBB(int32 InX, int32 InY);

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
	// ����
	// ��(0), �ϵ�(1), ��(2), ����(3), ��(4), ����(5), ��(6), �ϼ�(7) , ������(8��)
	const int32	NODIRECTION = 8;

	// �� ���
	UPROPERTY()
	UJPSHeap* OpenList;

	// ���� ���
	TDBitArray<int64> ClosedList;

	JPSCoord EndPos;

	TWeakObjectPtr<AJPSCollision> FieldCollision;
	int32 GridWidth = 0;
	int32 GridHeight = 0;
};
