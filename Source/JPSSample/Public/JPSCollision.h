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
	// 2D �׸����� �ʺ�
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	int32 Width;
	// 2D �׸����� ����
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	int32 Height;

private:
	static const int64 NPos = ~(0);	//	default npos == -1

	// X������ 2���� ��Ʈ�迭
	TDBitArray<int64> XBoundaryPoints;
	// Y������ 2���� ��Ʈ�迭
	TDBitArray<int64> YBoundaryPoints;
	// ���� �ٸ� 2���� ��Ʈ�迭�� ���� ������ ��Ʈ ������ ���ι���(�޸� ����) ���θ� �� �� �ֱ� ������ ���� ��Ī�Ǵ� ��Ʈ�迭 2������ ����Ѵ�

public:
	UPROPERTY()
	UJPSPath* JPSPathfinder;

};
