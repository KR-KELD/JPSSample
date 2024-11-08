// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

struct FDivResult
{
	int32 Quotient;
	int32 Remainder;
};

template <typename Ty>
class TDBitArray : public TArray<Ty>
{
public:
	// TArray�� ��� Ty�� ��Ʈ ��
	static const int32 NBITMASK = static_cast<int32>(8 * sizeof(Ty));
	// TyŸ���� ��Ʈ�� ��� 1�� ä���� ����
	static const Ty	FULLBITS = std::numeric_limits<Ty>::max();
	// TyŸ���� ��Ʈ�� ��� 0���� ä���� ����
	static const Ty	CLEARBITS = 0;
public:
	// �׸����� ���̸� Ty�� ũ��� ������
	FDivResult Divide(int32 InPos)
	{
		FDivResult Result;
		Result.Quotient = InPos / NBITMASK;
		Result.Remainder = InPos % NBITMASK;
		return Result;
	}
	// �������� �޾Ƽ� ������ ��Ʈ ��ġ�� ����Ʈ�Ͽ� ��Ʈ����ũ�� �����Ѵ�
	Ty DivMaskbits(const FDivResult& InDiv) { return (Ty)1 << InDiv.Remainder; }
	// 2���� ��ǥ�� 1���� pos�� ��ȯ�Ѵ�
	int32 ToOneDimensionPos(int32 InX, int32 InY) { return (int32)(InY * Bitswidths + InX); }

public:
	int32 GetBitsWidths() const { return Bitswidths; }
	int32 GetWordWidths() const { return Wordwidths; }
	int32 GetWidth() const { return Width; }
	int32 GetHeight() const { return Depth; }

	void Create(int32 InWidth, int32 InDepth, Ty InVal = 0)
	{
		// �׸����� �� ũ�⸦ Ty�� ��Ʈ ���� ��� ���� �����⸦ �Ѵ�
		FDivResult Div = Divide(InWidth);
		// �������� �ִٸ� �ø�
		if (Div.Remainder)
		{
			Div.Quotient++;
		}

		// 64��Ʈ ����
		// �� �࿡ �� ������ ��
		Wordwidths = Div.Quotient;
		// �� �࿡ �� ������ ��ü ��Ʈ ��
		Bitswidths = Wordwidths * NBITMASK;
		// ���� ���� �� X ���� ���� �� = �ʿ��� ������ �Ҵ�
		this->SetNum(Wordwidths * InDepth);

		// �׸����� ��
		Width = InWidth;
		// �׸����� ��
		Depth = InDepth;
	}

	bool Set(Ty* InData, int32 InCount)
	{
		// �����ŭ �迭 ����
		assert((int32)this->Num() == InCount);
		if ((int32)this->Num() < InCount)
		{
			return false;
		}

		FMemory::Memcpy(this->GetData(), InData, sizeof(Ty) * InCount);
		return true;
	}

	Ty GetValue(int32 InX, int32 InY)
	{
		// ��ǥ�� �����ϴ� ���Ҹ� ��ȯ
		int32 Pos = ToOneDimensionPos(InX, InY);
		if (Pos < 0 || Pos >= (static_cast<int32>(this->Num()) * NBITMASK))
		{
			return ~(0);
		}
		
		return (*this)[static_cast<int32>(Pos / NBITMASK)];
	}

	void Clear(Ty InVal = 0)
	{
		for (Ty& Element : *this)
		{
			Element = InVal;
		}
	}

	bool SetAt(int32 InX, int32 InY, bool InFlag)
	{
		// 2���� ��ǥ�� �޾Ƽ� �ش� ��ġ�� ��Ʈ�� �����Ѵ� (InFlag true => 1, InFlag false => 0)
		int32 Pos = ToOneDimensionPos(InX, InY);
		if (Pos < 0 || Pos >= (static_cast<int32>(this->Num()) * NBITMASK))
		{
			return false;
		}
		
		FDivResult Div = Divide(Pos);
		if (InFlag)
		{
			(*this)[Div.Quotient] |= DivMaskbits(Div);
		}
		else
		{
			(*this)[Div.Quotient] &= ~DivMaskbits(Div);
		}
		return true;
	}

	bool IsSet(int32 InX, int32 InY)
	{
		// 2���� ��ǥ�� �޾Ƽ� �ش� ��ġ�� ��Ʈ�� 1���� 0���� Ȯ��
		int32 Pos = ToOneDimensionPos(InX, InY);
		if (Pos < 0 || Pos >= (static_cast<int32>(this->Num()) * NBITMASK))
		{
			return true;
		}

		FDivResult Div = Divide(Pos);
		return ((*this)[Div.Quotient] & DivMaskbits(Div)) ? true : false;
	}

public:
	// ���� ����(Ty ũ���� ���)
	int32 Bitswidths = 0;
	// ���� ���� ����(�� �࿡ TyŸ�� ���Ұ� 3�� ���ٸ� 3)
	int32 Wordwidths = 0;

	// �׸����� ��
	int32 Width = 0;
	// �׸����� ��
	int32 Depth = 0;
};