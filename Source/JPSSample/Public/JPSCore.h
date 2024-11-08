// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "JPSCore.generated.h"

struct JPSCoord
{
	int32 X = -1, Y = -1;

public:
	JPSCoord() = default;
	JPSCoord(int32 InX, int32 InY) : X(InX), Y(InY) { }

	void Add(const JPSCoord& InRhs) { X += InRhs.X;	Y += InRhs.Y; }
	void Clear() { X = -1;	Y = -1; }
	bool IsEmpty() const { return (X == -1 && Y == -1); }

	bool operator ==(const JPSCoord& InCoord) const { return ((InCoord.X == X) && (InCoord.Y == Y)); }
	bool operator !=(const JPSCoord& InCoord) const { return !(*this == InCoord); }

	float GetOctileDistance(const JPSCoord& InRhs)
	{
		// �׸���ȿ����� 8���� �̵��̱⶧���� ��Ȯ�� �Ÿ� �ڽ�Ʈ ����� ���� octile distance�� ����Ѵ�
		int32 AbsX = FMath::Abs(X - InRhs.X);
		int32 AbsY = FMath::Abs(Y - InRhs.Y);

		int32 DiagDist = FMath::Min(AbsX, AbsY);
		int32 StraightDist = FMath::Max(AbsX, AbsY) - DiagDist;
		return DiagDist * 1.414213562373095f + StraightDist;
	}
};

USTRUCT()
struct FJPSNode
{
	GENERATED_BODY()

	TSharedPtr<FJPSNode> Parent = nullptr;
	JPSCoord Pos;				// Compare Same Position
	char CardinalDir = 0;		// �̵�����
	float Score = 0.0f;			// ���۳����� ���� �������� �̵����
	float Heuri = 0.0f;			// ���� ��忡�� ��ǥ �������� �������
	float Total = 0.0f;			// Score�� Heuri�� �� Total Score�� ���� Node �� P.Queue ���� ���� �ö󰣴�

public:

	void Clear()
	{
		Parent.Reset();
		Pos.Clear();
		CardinalDir = 0;
		Score = 0.0f;
		Heuri = 0.0f;
		Total = 0.0f;
	}

	void Set(TSharedPtr<FJPSNode> InParent, const JPSCoord& InPos, const JPSCoord& InEnd, const char InCardinalDir)
	{
		Parent = InParent;
		Pos = InPos;
		CardinalDir = InCardinalDir;
		if (Parent.IsValid())
		{
			Score = Pos.GetOctileDistance(Parent->Pos) + Parent->Score;
		}
		else
		{
			Score = 0;
		}
		Heuri = Pos.GetOctileDistance(InEnd);
		Total = Score + Heuri;
	}
};


UCLASS()
class UJPSHeap : public UObject
{
	GENERATED_BODY()
private:
	TArray<TSharedPtr<FJPSNode>> Heap;

public:

	void Insert(TSharedPtr<FJPSNode> InValue)
	{
		Heap.Add(InValue);
		ShiftUp(Heap.Num() - 1);
	}

	bool InsertSmaller(TSharedPtr<FJPSNode> InValue)
	{
		// ���� ��ġ�ε� �ٸ� �ڽ�Ʈ�� ���� ����� ���Ž����ش�
		if (!InValue.IsValid())
		{
			return false;
		}

		int32 HeapSize = Heap.Num();

		for (int32 Node = 0; Node < HeapSize; Node++)
		{
			TSharedPtr<FJPSNode> CurrentNode = Heap[Node];
			if (!CurrentNode.IsValid())
			{
				continue;
			}

			if (InValue->Pos != CurrentNode->Pos)
			{
				continue;
			}

			if (InValue->Total >= CurrentNode->Total)
			{
				return false;
			}
			else
			{
				Heap[Node] = InValue;
				ShiftUp(Node);
				return true;
			}
		}

		return false;
	}

	TSharedPtr<FJPSNode> PopMin()
	{
		if (Heap.Num() == 0)
		{
			// ���� ��� �ִ� ��� ó�� �ʿ�
			return nullptr;
		}

		TSharedPtr<FJPSNode> Result = Heap[0];
		Heap[0] = Heap.Last();
		Heap.Pop();
		ShiftDown(0);
		return Result;
	}

	const TSharedPtr<FJPSNode> GetMin() const
	{
		if (Heap.Num() > 0)
		{
			return Heap[0];
		}
		else
		{
			return nullptr;
		}
	}

	int32 GetCount() const { return (int32)Heap.Num(); }
	void ClearHeap() { Heap.Empty(); }

private:

	void BuildHeap()
	{
		for (int32 i = (Heap.Num() / 2) - 1; i >= 0; --i)
		{
			ShiftDown(i);
		}
	}

	void ShiftDown(int32 InParent)
	{
		int32 HeapSize = Heap.Num();
		while (true)
		{
			int32 Left = InParent * 2 + 1;
			if (Left >= HeapSize)
			{
				break;
			}
			int32 Right = Left + 1;
			int32 SmallestChild = Left;

			if (Right < HeapSize && Heap[Right]->Total < Heap[Left]->Total)
			{
				SmallestChild = Right;
			}

			if (Heap[InParent]->Total <= Heap[SmallestChild]->Total)
			{
				break;
			}

			Heap.Swap(InParent, SmallestChild);
			InParent = SmallestChild;
		}
	}

	void ShiftUp(int32 InNode)
	{
		while (InNode > 0)
		{
			int32 Parent = (InNode - 1) / 2;

			if (Heap[InNode]->Total >= Heap[Parent]->Total)
			{
				return;
			}

			Heap.Swap(InNode, Parent);
			InNode = Parent;
		}
	}
};
