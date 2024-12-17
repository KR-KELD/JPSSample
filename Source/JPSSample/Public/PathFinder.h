// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "PathFinder.generated.h"

class AMaze;
class AJPSCollision;
class AAStarCollision;

UCLASS()
class JPSSAMPLE_API APathFinder : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	APathFinder();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "PathFinding")
	void BuildMap();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "PathFinding")
	void CalcCollision();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "PathFinding")
	void PathFinding();

	FVector GetNodeLocation(int32 InX, int32 InY);
	FIntPoint LocationToCoord(FVector InLocation);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:

	UPROPERTY(EditAnywhere)
	int32 Width;
	UPROPERTY(EditAnywhere)
	int32 Height;

	// 셀의 가로 길이
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	float IntervalX;
	// 셀의 세로 길이
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	float IntervalY;
	// 셀 검사의 높이 제한
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	float HeightLimit;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	bool GraphAStarDebug;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	bool JPSDebug;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "JPSArea")
	bool NavmeshDebug;

public:

	UPROPERTY(EditAnywhere)
	AAStarCollision* AStarCollision;

	UPROPERTY(EditAnywhere)
	AJPSCollision* JPSCollision;

	UPROPERTY(EditAnywhere)
	AMaze* MazeActor;

	UPROPERTY(EditAnywhere)
	AActor* NavMeshVolumeActor;

	UPROPERTY(EditAnywhere)
	AActor* StartActor;

	UPROPERTY(EditAnywhere)
	AActor* EndActor;
	
};
