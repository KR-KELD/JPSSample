// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "TDBitArray.h"
#include "JPSCore.h"

#include "JPSCollision.generated.h"

class UJPSPath;

UCLASS()
class AJPSCollision : public AActor
{
	GENERATED_BODY()

public:
	AJPSCollision();

	virtual void BeginPlay() override;
public:
	bool CreateMap();

	int32 GetWidth() const { return Width; }
	int32 GetHeight() const{ return Height; }
	void SetWidth(const int32& InWidth) { Width = InWidth; }
	void SetHeight(const int32& InHeight) { Height = InHeight; }
	bool IsOutBound(int32 InX, int32 InY) const;
	bool IsCollision(int32 InX, int32 InY);

	void SetAt(int32 InX, int32 InY);
	void ClearAt(int32 InX, int32 InY);

	int32 GetCloseValue(int32 InX, int32 InY, bool IsXaxis, bool IsForward);
	int32 GetOpenValue(int32 InX, int32 InY, bool IsXaxis, bool IsForward);

	void BuildMap();

	void FindPath(FIntPoint InStartCoord, FIntPoint InEndCoord, TArray<FIntPoint>& OutResultPos);

private:
	int32 GetPosX(int32 InX, int32 InY);
	int32 GetPosY(int32 InX, int32 InY);

	static bool BitScanReverse64(unsigned long& InIndex, uint64 InWord);
	static bool BitScanForward64(unsigned long& InIndex, uint64 InWord);

public:
	// 2D 그리드의 너비
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	int32 Width;
	// 2D 그리드의 높이
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	int32 Height;

private:
	static const int64 NPos = ~(0);	//	default npos == -1

	// X방향의 2차원 비트배열
	TDBitArray<int64> XBoundaryPoints;
	// Y방향의 2차원 비트배열
	TDBitArray<int64> YBoundaryPoints;
	// 서로 다른 2차원 비트배열을 쓰는 이유는 비트 접근은 가로방향(메모리 연속) 으로만 할 수 있기 때문에 서로 대칭되는 비트배열 2가지를 사용한다

public:
	UPROPERTY()
	UJPSPath* JPSPathfinder;

};
