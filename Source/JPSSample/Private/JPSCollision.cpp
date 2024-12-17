// Fill out your copyright notice in the Description page of Project Settings.


#include "JPSCollision.h"
#include "JPSPath.h"

#include "TDBitArray.h"
#include "NavigationSystem.h"
#include "AI/Navigation/NavigationTypes.h"


AJPSCollision::AJPSCollision()
{
	JPSPathfinder = CreateDefaultSubobject<UJPSPath>(TEXT("JPSPath"));
	Width = 32;
	Height = 32;
}

void AJPSCollision::BeginPlay()
{
	Super::BeginPlay();
}

bool AJPSCollision::CreateMap()
{
	// X�� ���� 2���� ��Ʈ�迭 �ʱ�ȭ
	XBoundaryPoints.Empty();
	XBoundaryPoints.Create(Width, Height);
	// Y�� ���� 2���� ��Ʈ�迭 �ʱ�ȭ
	YBoundaryPoints.Empty();
	YBoundaryPoints.Create(Height, Width);

	// ��Ʈ�迭�� ���� ũ�⺸�� ���� ���̰� ���� �����Ǳ� ������ ������� �ʴ� �κ��� �浹�������� �ٲ۴�
	for (int GridX = Width; GridX < XBoundaryPoints.GetBitsWidths(); GridX++)
	{
		for (int GridY = Height; GridY < YBoundaryPoints.GetBitsWidths(); GridY++)
		{
			SetAt(GridX, GridY);
		}
	}
	return true;
}

bool AJPSCollision::IsOutBound(int32 InX, int32 InY) const
{
	if (InX < 0)
	{
		return true;
	}
	if (InY < 0)
	{
		return true;
	}
	if (InX >= Width)
	{
		return true;
	}
	if (InY >= Height)
	{
		return true;
	}
	return false;
}

bool AJPSCollision::IsCollision(int32 InX, int32 InY)
{
	return XBoundaryPoints.IsSet(InX, InY);
}

void AJPSCollision::SetAt(int32 InX, int32 InY)
{
	XBoundaryPoints.SetAt(InX, InY, true);
	YBoundaryPoints.SetAt(InY, InX, true);
}

void AJPSCollision::ClearAt(int32 InX, int32 InY)
{
	XBoundaryPoints.SetAt(InX, InY, false);
	YBoundaryPoints.SetAt(InY, InX, false);
}

int32 AJPSCollision::GetCloseValue(int32 InX, int32 InY, bool IsXaxis, bool IsForward)
{
	// uint64�� ��Ʈ 64���� ��� 1�� ���º��� �� ������ ��Ʈ���� �� ĭ�� ����Ʈ�Ͽ� 10000000~������ ������ �迭 (�����)
	// 1111 -> 1110 -> 1100 -> 1000
	static const uint64 PlusTable[] = 
	{
		18446744073709551615ULL,		18446744073709551614ULL,		18446744073709551612ULL,		18446744073709551608ULL,
		18446744073709551600ULL,		18446744073709551584ULL,		18446744073709551552ULL,		18446744073709551488ULL,
		18446744073709551360ULL,		18446744073709551104ULL,		18446744073709550592ULL,		18446744073709549568ULL,
		18446744073709547520ULL,		18446744073709543424ULL,		18446744073709535232ULL,		18446744073709518848ULL,
		18446744073709486080ULL,		18446744073709420544ULL,		18446744073709289472ULL,		18446744073709027328ULL,
		18446744073708503040ULL,		18446744073707454464ULL,		18446744073705357312ULL,		18446744073701163008ULL,
		18446744073692774400ULL,		18446744073675997184ULL,		18446744073642442752ULL,		18446744073575333888ULL,
		18446744073441116160ULL,		18446744073172680704ULL,		18446744072635809792ULL,		18446744071562067968ULL,
		18446744069414584320ULL,		18446744065119617024ULL,		18446744056529682432ULL,		18446744039349813248ULL,
		18446744004990074880ULL,		18446743936270598144ULL,		18446743798831644672ULL,		18446743523953737728ULL,
		18446742974197923840ULL,		18446741874686296064ULL,		18446739675663040512ULL,		18446735277616529408ULL,
		18446726481523507200ULL,		18446708889337462784ULL,		18446673704965373952ULL,		18446603336221196288ULL,
		18446462598732840960ULL,		18446181123756130304ULL,		18445618173802708992ULL,		18444492273895866368ULL,
		18442240474082181120ULL,		18437736874454810624ULL,		18428729675200069632ULL,		18410715276690587648ULL,
		18374686479671623680ULL,		18302628885633695744ULL,		18158513697557839872ULL,		17870283321406128128ULL,
		17293822569102704640ULL,		16140901064495857664ULL,		13835058055282163712ULL,		9223372036854775808ULL,
	};

	// ���� �ݴ�� �����ʺ��� ���������� ����
	// 0001 -> 0011 -> 0111 -> 1111
	static const uint64 MinusTable[] = 
	{
		1ULL,						3ULL,						7ULL,						15ULL,
		31ULL,						63ULL,						127ULL,						255ULL,
		511ULL,						1023ULL,					2047ULL,					4095ULL,
		8191ULL,					16383ULL,					32767ULL,					65535ULL,
		131071ULL,					262143ULL,					524287ULL,					1048575ULL,
		2097151ULL,					4194303ULL,					8388607ULL,					16777215ULL,
		33554431ULL,				67108863ULL,				134217727ULL,				268435455ULL,
		536870911ULL,				1073741823ULL,				2147483647ULL,				4294967295ULL,
		8589934591ULL,				17179869183ULL,				34359738367ULL,				68719476735ULL,
		137438953471ULL,			274877906943ULL,			549755813887ULL,			1099511627775ULL,
		2199023255551ULL,			4398046511103ULL,			8796093022207ULL,			17592186044415ULL,
		35184372088831ULL,			70368744177663ULL,			140737488355327ULL,			281474976710655ULL,
		562949953421311ULL,			1125899906842623ULL,		2251799813685247ULL,		4503599627370495ULL,
		9007199254740991ULL,		18014398509481983ULL,		36028797018963967ULL,		72057594037927935ULL,
		144115188075855871ULL,		288230376151711743ULL,		576460752303423487ULL,		1152921504606846975ULL,
		2305843009213693951ULL,		4611686018427387903ULL,		9223372036854775807ULL,		18446744073709551615ULL,
	};


	int32 MaxValue = IsXaxis ? Width : Height;
	TDBitArray<int64>& MaskBound = IsXaxis ? XBoundaryPoints : YBoundaryPoints;
	int32 Variable = IsXaxis ? InX : InY;
	int32 NBitmask = IsXaxis ? XBoundaryPoints.NBITMASK : YBoundaryPoints.NBITMASK;
	if (IsForward)
	{
		// ���� ���� ��
		int32 MaxWidths = MaskBound.GetWordWidths();
		for (int i = 0; i < MaxWidths; ++i)
		{
			// ��ǥ�� 1���� ��ġ�� ��ȯ, ���� ������ �˻��ϱ⶧���� i�� ������ ��Ʈ����ŭ �����ش�
			int32 Pos = IsXaxis ? GetPosX(InX + i * NBitmask, InY) : GetPosY(InX, InY + i * NBitmask);
			if (Pos == NPos)
			{
				return MaxValue;
			}

			// ���õ� ����
			int64 Value = MaskBound[Pos / NBitmask];
			// ���õ� ���Ұ� ������ġ�� ���Ե� ���Ҷ�� �÷������̺�� and������ �ؼ� ������ġ ������ ������ ��� 0���� �ٲ۴�
			// ��Ʈ���� 64�̰� ��ũ�Ⱑ 100�̸� �� �࿡ ���Ҵ� 128�� x��ǥ�� 10�̸� 10��° ���� 
			if (i == 0)
			{
				Value &= PlusTable[(Variable % NBitmask)];
			}

			unsigned long index = 0;
			if (AJPSCollision::BitScanForward64(index, Value))
			{
				// ���� ����� �浹���� ��ȯ
				// 00000x000000 000000000��11
				return (Variable - (Variable % NBitmask) + i * NBitmask) + index;
			}
		}
		// ���ٸ� ������ ��ȯ
		return MaxValue;
	}
	else
	{
		int32 MaxWidths = MaskBound.GetWordWidths();
		for (int i = 0; i < MaxWidths; ++i)
		{
			int32 Pos = IsXaxis ? GetPosX(InX + i * NBitmask, InY) : GetPosY(InX, InY + i * NBitmask);
			if (Pos == NPos)
			{
				return -1;
			}

			int64 Value = MaskBound[Pos / NBitmask];
			if (i == 0)
			{
				Value &= MinusTable[(Variable % NBitmask)];
			}

			unsigned long index = 0;
			if (AJPSCollision::BitScanReverse64(index, Value))
			{
				return (Variable - (Variable % NBitmask) - i * NBitmask) + index;
			}
		}
		return -1;
	}
}

int32 AJPSCollision::GetOpenValue(int32 InX, int32 InY, bool IsXaxis, bool IsForward)
{
	static const uint64 PlusTable[] =
	{
		18446744073709551615ULL,		18446744073709551614ULL,		18446744073709551612ULL,		18446744073709551608ULL,
		18446744073709551600ULL,		18446744073709551584ULL,		18446744073709551552ULL,		18446744073709551488ULL,
		18446744073709551360ULL,		18446744073709551104ULL,		18446744073709550592ULL,		18446744073709549568ULL,
		18446744073709547520ULL,		18446744073709543424ULL,		18446744073709535232ULL,		18446744073709518848ULL,
		18446744073709486080ULL,		18446744073709420544ULL,		18446744073709289472ULL,		18446744073709027328ULL,
		18446744073708503040ULL,		18446744073707454464ULL,		18446744073705357312ULL,		18446744073701163008ULL,
		18446744073692774400ULL,		18446744073675997184ULL,		18446744073642442752ULL,		18446744073575333888ULL,
		18446744073441116160ULL,		18446744073172680704ULL,		18446744072635809792ULL,		18446744071562067968ULL,
		18446744069414584320ULL,		18446744065119617024ULL,		18446744056529682432ULL,		18446744039349813248ULL,
		18446744004990074880ULL,		18446743936270598144ULL,		18446743798831644672ULL,		18446743523953737728ULL,
		18446742974197923840ULL,		18446741874686296064ULL,		18446739675663040512ULL,		18446735277616529408ULL,
		18446726481523507200ULL,		18446708889337462784ULL,		18446673704965373952ULL,		18446603336221196288ULL,
		18446462598732840960ULL,		18446181123756130304ULL,		18445618173802708992ULL,		18444492273895866368ULL,
		18442240474082181120ULL,		18437736874454810624ULL,		18428729675200069632ULL,		18410715276690587648ULL,
		18374686479671623680ULL,		18302628885633695744ULL,		18158513697557839872ULL,		17870283321406128128ULL,
		17293822569102704640ULL,		16140901064495857664ULL,		13835058055282163712ULL,		9223372036854775808ULL,
	};

	static const uint64 MinusTable[] =
	{
		1ULL,						3ULL,						7ULL,						15ULL,
		31ULL,						63ULL,						127ULL,						255ULL,
		511ULL,						1023ULL,					2047ULL,					4095ULL,
		8191ULL,					16383ULL,					32767ULL,					65535ULL,
		131071ULL,					262143ULL,					524287ULL,					1048575ULL,
		2097151ULL,					4194303ULL,					8388607ULL,					16777215ULL,
		33554431ULL,				67108863ULL,				134217727ULL,				268435455ULL,
		536870911ULL,				1073741823ULL,				2147483647ULL,				4294967295ULL,
		8589934591ULL,				17179869183ULL,				34359738367ULL,				68719476735ULL,
		137438953471ULL,			274877906943ULL,			549755813887ULL,			1099511627775ULL,
		2199023255551ULL,			4398046511103ULL,			8796093022207ULL,			17592186044415ULL,
		35184372088831ULL,			70368744177663ULL,			140737488355327ULL,			281474976710655ULL,
		562949953421311ULL,			1125899906842623ULL,		2251799813685247ULL,		4503599627370495ULL,
		9007199254740991ULL,		18014398509481983ULL,		36028797018963967ULL,		72057594037927935ULL,
		144115188075855871ULL,		288230376151711743ULL,		576460752303423487ULL,		1152921504606846975ULL,
		2305843009213693951ULL,		4611686018427387903ULL,		9223372036854775807ULL,		18446744073709551615ULL,
	};


	int32 MaxValue = IsXaxis ? Width : Height;
	TDBitArray<int64>& MaskBound = IsXaxis ? XBoundaryPoints : YBoundaryPoints;
	int32 Variable = IsXaxis ? InX : InY;
	int32 NBitmask = IsXaxis ? XBoundaryPoints.NBITMASK : YBoundaryPoints.NBITMASK;
	if (IsForward)
	{
		int32 MaxWidths = MaskBound.GetWordWidths();
		for (int i = 0; i < MaxWidths; ++i)
		{
			int32 Pos = IsXaxis ? GetPosX(InX + i * NBitmask, InY) : GetPosY(InX, InY + i * NBitmask);
			if (Pos == NPos)
			{
				return MaxValue;
			}

			// ���簪�� �ݴ� ��Ʈ���� �ο��ؼ� ���� ����� ���������� Ž��
			int64 Value = ~MaskBound[Pos / NBitmask];
			if (i == 0)
			{
				Value &= PlusTable[(Variable % NBitmask)];
			}

			unsigned long index = 0;
			if (AJPSCollision::BitScanForward64(index, Value))
			{
				return (Variable - (Variable % NBitmask) + i * NBitmask) + index;
			}
		}
		return MaxValue;
	}
	else
	{
		int32 MaxWidths = MaskBound.GetWordWidths();
		for (int i = 0; i < MaxWidths; ++i)
		{
			int32 Pos = IsXaxis ? GetPosX(InX + i * NBitmask, InY) : GetPosY(InX, InY + i * NBitmask);
			if (Pos == NPos)
			{
				return -1;
			}

			int64 Value = ~MaskBound[Pos / NBitmask];
			if (i == 0)
			{
				Value &= MinusTable[(Variable % NBitmask)];
			}

			unsigned long index = 0;
			if (AJPSCollision::BitScanReverse64(index, Value))
			{
				return (Variable - (Variable % NBitmask) - i * NBitmask) + index;
			}
		}
		return -1;
	}
}

void AJPSCollision::BuildMap()
{
	CreateMap();

	if (IsValid(JPSPathfinder))
	{
		JPSPathfinder->SetMap(this);
	}
}

void AJPSCollision::FindPath(FIntPoint InStartCoord, FIntPoint InEndCoord, TArray<FIntPoint>& OutResultPos)
{
	if (!IsValid(JPSPathfinder))
	{
		UE_LOG(LogTemp, Error, TEXT("Not Exist JPSPathfinder"));
		return;
	}

	JPSPathfinder->Search(InStartCoord, InEndCoord, OutResultPos);
}

int32 AJPSCollision::GetPosX(int32 InX, int32 InY)
{
	if (InX < 0 || InX >= Width || InY < 0 || InY >= Height)
	{
		return NPos;
	}
	return InX + InY * XBoundaryPoints.GetBitsWidths();
}

int32 AJPSCollision::GetPosY(int32 InX, int32 InY)
{
	if (InX < 0 || InX >= Width || InY < 0 || InY >= Height)
	{
		return NPos;
	}
	return InX * YBoundaryPoints.GetBitsWidths() + InY;
}

bool AJPSCollision::BitScanReverse64(unsigned long& InIndex, uint64 InWord)
{
	// �� ������ ��Ʈ �ε������� ���� ó�� ��Ʈ�� 1�� ���� ã�� �Լ�
	unsigned long Num = 63;
	if (!InWord)
	{
		return false;
	}

	if (!(InWord & 0xffffffff00000000ULL)) 
	{
		InWord <<= 32;
		Num -= 32;
	}

	if (!(InWord & 0xffff000000000000ULL)) 
	{
		InWord <<= 16;
		Num -= 16;
	}

	if (!(InWord & 0xff00000000000000ULL))
	{
		InWord <<= 8;
		Num -= 8;
	}

	if (!(InWord & 0xf000000000000000ULL))
	{
		InWord <<= 4;
		Num -= 4;
	}

	if (!(InWord & 0xc000000000000000ULL)) 
	{
		InWord <<= 2;
		Num -= 2;
	}

	if (!(InWord & 0x8000000000000000ULL))
	{
		InWord <<= 1;
		Num -= 1;
	}

	InIndex = Num;
	return true;
}

bool AJPSCollision::BitScanForward64(unsigned long& InIndex, uint64 InWord)
{
	// �� ó�� ��Ʈ �ε������� ���� ó�� ��Ʈ�� 1�� ���� ã�� �Լ�
	unsigned long Num = 0;
	if (!InWord)
	{
		return false;
	}
		
	if ((InWord & 0xffffffffULL) == 0) 
	{
		Num += 32;
		InWord >>= 32;
	}

	if ((InWord & 0xffffULL) == 0) 
	{
		Num += 16;
		InWord >>= 16;
	}

	if ((InWord & 0xffULL) == 0)
	{
		Num += 8;
		InWord >>= 8;
	}

	if ((InWord & 0xfULL) == 0)
	{
		Num += 4;
		InWord >>= 4;
	}

	if ((InWord & 0x3ULL) == 0) 
	{
		Num += 2;
		InWord >>= 2;
	}

	if ((InWord & 0x1ULL) == 0)
	{
		Num += 1;
	}

	InIndex = Num;
	return true;
}
