// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/PlayerController.h"
#include "MyPlayerController.generated.h"

class UJPSPath;
/**
 * 
 */
UCLASS()
class JPSSAMPLE_API AMyPlayerController : public APlayerController
{
	GENERATED_BODY()
	
public:
	AMyPlayerController();

	virtual void BeginPlay() override;

public:
	UFUNCTION(BlueprintCallable)
	bool JPSPathfind();
public:
	UPROPERTY()
	UJPSPath* JPSPathfinder;

};
