// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

#include "PathFinder.generated.h"

class AMaze;
class AJPSCollision;
class AAStarCollision;

UENUM(BlueprintType)
enum class EMapType : uint8
{
	None	UMETA(DisplayName = "No Collision"),
	Random	UMETA(DisplayName = "Random Obstacles"),
	Maze	UMETA(DisplayName = "Maze"),
	Room	UMETA(DisplayName = "Rooms"),
	Block	UMETA(DisplayName = "Block"),
	Navmesh	UMETA(DisplayName = "Follow Navmesh"),
};

struct FMyBox
{
	int32 X;
	int32 Y;
	int32 W;
	int32 H;
	bool IsCollision;
};

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
	void InitData();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "PathFinding")
	void PathFinding();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "PathFinding")
	void ShowMap();

	UFUNCTION(BlueprintCallable, CallInEditor, Category = "PathFinding")
	void PathFindingSimulate();

	FVector GetNodeLocation(int32 InX, int32 InY, bool InCheckNavmesh = true);
	FIntPoint LocationToCoord(FVector InLocation);

	bool OverlapsMyBox(const FMyBox& InBoxA, const FMyBox& InBoxB);
	TArray<TArray<uint8>> GenerateMaze(int32 InMapSize);
	TArray<TArray<uint8>> GenerateRoomsMap(int32 InMapSize);
	TArray<TArray<uint8>> GenerateBlock(int32 InMapSize);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:

	UPROPERTY(EditAnywhere)
	int32 Width;
	UPROPERTY(EditAnywhere)
	int32 Height;

	// 셀의 가로 길이
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	float IntervalX;
	// 셀의 세로 길이
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	float IntervalY;
	// 셀 검사의 높이 제한
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	float HeightLimit;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	bool GraphAStarDebug;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	bool JPSDebug;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	bool NavmeshDebug;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	float ShowMapTime;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	EMapType MapType;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Pathfinder")
	int32 PathFindingSimulateCount;

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
	
private:
	FIntPoint StartCoord;
	FIntPoint EndCoord;

	int32 AStarCount;
	int32 JPSCount;
	double AStarTime;
	double JPSTime;
};
