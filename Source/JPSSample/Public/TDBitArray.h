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
	// TArray의 요소 Ty의 비트 수
	static const int32 NBITMASK = static_cast<int32>(8 * sizeof(Ty));
	// Ty타입의 비트가 모두 1로 채워진 변수
	static const Ty	FULLBITS = std::numeric_limits<Ty>::max();
	// Ty타입의 비트가 모두 0으로 채워진 변수
	static const Ty	CLEARBITS = 0;
public:
	// 그리드의 길이를 Ty의 크기로 나눈다
	FDivResult Divide(int32 InPos)
	{
		FDivResult Result;
		Result.Quotient = InPos / NBITMASK;
		Result.Remainder = InPos % NBITMASK;
		return Result;
	}
	// 나머지를 받아서 원소의 비트 위치로 시프트하여 비트마스크를 생성한다
	Ty DivMaskbits(const FDivResult& InDiv) { return (Ty)1 << InDiv.Remainder; }
	// 2차원 좌표를 1차원 pos로 전환한다
	int32 ToOneDimensionPos(int32 InX, int32 InY) { return (int32)(InY * Bitswidths + InX); }

public:
	int32 GetBitsWidths() const { return Bitswidths; }
	int32 GetWordWidths() const { return Wordwidths; }
	int32 GetWidth() const { return Width; }
	int32 GetHeight() const { return Depth; }

	void Create(int32 InWidth, int32 InDepth, Ty InVal = 0)
	{
		// 그리드의 행 크기를 Ty의 비트 수에 담기 위해 나누기를 한다
		FDivResult Div = Divide(InWidth);
		// 나머지가 있다면 올림
		if (Div.Remainder)
		{
			Div.Quotient++;
		}

		// 64비트 기준
		// 한 행에 들어갈 원소의 수
		Wordwidths = Div.Quotient;
		// 한 행에 들어갈 원소의 전체 비트 수
		Bitswidths = Wordwidths * NBITMASK;
		// 행의 원소 수 X 열의 원소 수 = 필요한 사이즈 할당
		this->SetNum(Wordwidths * InDepth);

		// 그리드의 행
		Width = InWidth;
		// 그리드의 열
		Depth = InDepth;
	}

	bool Set(Ty* InData, int32 InCount)
	{
		// 사이즈만큼 배열 복사
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
		// 좌표가 존재하는 원소를 반환
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
		// 2차원 좌표를 받아서 해당 위치의 비트를 변경한다 (InFlag true => 1, InFlag false => 0)
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
		// 2차원 좌표를 받아서 해당 위치의 비트가 1인지 0인지 확인
		int32 Pos = ToOneDimensionPos(InX, InY);
		if (Pos < 0 || Pos >= (static_cast<int32>(this->Num()) * NBITMASK))
		{
			return true;
		}

		FDivResult Div = Divide(Pos);
		return ((*this)[Div.Quotient] & DivMaskbits(Div)) ? true : false;
	}

public:
	// 행의 길이(Ty 크기의 배수)
	int32 Bitswidths = 0;
	// 행의 원소 갯수(한 행에 Ty타입 원소가 3개 들어간다면 3)
	int32 Wordwidths = 0;

	// 그리드의 행
	int32 Width = 0;
	// 그리드의 열
	int32 Depth = 0;
};