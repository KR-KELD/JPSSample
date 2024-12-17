// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "GraphAStar.h"

#include "AStarCollision.generated.h"

class AAStarCollision;
typedef FGraphAStar<AAStarCollision> FGraph;
typedef FGraphAStarDefaultNode<AAStarCollision> FSearchNode;
typedef FIntPoint FGridNodeRef;

UCLASS()
class JPSSAMPLE_API AAStarCollision : public AActor
{
	GENERATED_BODY()
public:

	int32 Width;
	int32 Height;
	TArray<bool> NodeAccessibility;

	void BuildMap(int32 InWidth, int32 InHeight)
	{
		NodeAccessibility.Empty();
		Width = InWidth;
		Height = InHeight;
		NodeAccessibility.Init(true, Width * Height);
	}
	// FGraphAStar: TGraph interface
	typedef FGridNodeRef FNodeRef; //= FIntPoint
	int32 GetNeighbourCount(FNodeRef NodeRef) const
	{
		return 8;
	}

	bool IsValidRef(FNodeRef NodeRef) const
	{
		if (NodeRef.X >= 0 && NodeRef.X < Width && NodeRef.Y >= 0 && NodeRef.Y < Height && IsNodeAccessible(NodeRef))
		{
			return true;
		}
		return false;
	}

	// ����� �̵� ���� ���� Ȯ��
	bool IsNodeAccessible(FNodeRef NodeRef) const
	{
		int32 Index = NodeRef.Y * Width + NodeRef.X;

		if (NodeAccessibility.IsValidIndex(Index))
		{
			return NodeAccessibility[Index];
		}
		return false;
	}

	// ����� �̵� ���� ���� ����
	void SetNodeAccessibility(int32 InX, int32 InY, bool bAccessible)
	{
		int32 Index = InY * Width + InX;
		if (NodeAccessibility.IsValidIndex(Index))
		{
			NodeAccessibility[Index] = bAccessible;
		}
	}

	FNodeRef GetNeighbour(const FSearchNode& NodeRef, const int32 NeighbourIndex) const
	{
		static const FIntPoint Directions[] = { FIntPoint(1, 0), FIntPoint(1, -1), FIntPoint(0, -1), FIntPoint(-1, -1), FIntPoint(-1, 0), FIntPoint(-1, 1), FIntPoint(0, 1), FIntPoint(1, 1) };
		return NodeRef.NodeRef + Directions[NeighbourIndex];
	}

};

struct FGridQueryFilter
{
	float HeuristicScale = 1.0f;

	FVector::FReal GetHeuristicScale() const { return HeuristicScale; }

	// Octile Distance�� ����Ͽ� ���� �Ÿ� ���
	FVector::FReal GetHeuristicCost(const FSearchNode& StartNode, const FSearchNode& EndNode) const
	{
		int32 Dx = FMath::Abs(StartNode.NodeRef.X - EndNode.NodeRef.X);
		int32 Dy = FMath::Abs(StartNode.NodeRef.Y - EndNode.NodeRef.Y);
		return FMath::Max(Dx, Dy) + (1.41421356237f - 1) * FMath::Min(Dx, Dy);
	}

	// ���� �̵� ��� ���
	FVector::FReal GetTraversalCost(const FSearchNode& StartNode, const FSearchNode& EndNode) const
	{
		FIntPoint Delta = EndNode.NodeRef - StartNode.NodeRef;
		return (FMath::Abs(Delta.X) + FMath::Abs(Delta.Y) > 1) ? 1.41421356237f : 1.0f;
	}

	// �̵� ���� ���� Ȯ��
	bool IsTraversalAllowed(FGridNodeRef NodeA, FGridNodeRef NodeB) const
	{
		return true; // ��� �̵� ���
	}

	// �κ� ��θ� ������� ����
	bool WantsPartialSolution() const { return false; }
};

