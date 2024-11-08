// Fill out your copyright notice in the Description page of Project Settings.

#include "JPSPath.h"

UJPSPath::UJPSPath()
{
	OpenList = CreateDefaultSubobject<UJPSHeap>(TEXT("JPSHeap"));
}

void UJPSPath::SetMap(AJPSCollision* InFieldCollision)
{
	FieldCollision = InFieldCollision;
	if (!FieldCollision.IsValid())
	{
		return;
	}

	GridWidth = InFieldCollision->GetWidth();
	GridHeight = InFieldCollision->GetHeight();
	ClosedList.Create(GridWidth, GridHeight);
}

void UJPSPath::DestroyMap()
{
	FieldCollision = nullptr;
	GridWidth = 0;
	GridHeight = 0;
	ClosedList.Clear();
}

bool UJPSPath::Search(FVector InStartLoc, FVector InEndLoc, TArray<FVector>& OutResultPos)
{
	if (!FieldCollision.IsValid())
	{
		return false;
	}

	TPair<int32, int32> Start = FieldCollision->GetGridLocation(InStartLoc);
	TPair<int32, int32> End = FieldCollision->GetGridLocation(InEndLoc);

	//���� üũ
	if ((Start.Key < 0 || Start.Key >= GridWidth) ||
		(Start.Value < 0 || Start.Value >= GridHeight) ||
		(End.Key < 0 || End.Key >= GridWidth) ||
		(End.Value < 0 || End.Value >= GridHeight) ||
		(Start.Key == End.Key && Start.Value == End.Value))
	{
		return false;
	}

	TArray<JPSCoord> PathResults;
	EndPos.X = End.Key;
	EndPos.Y = End.Value;
	OutResultPos.Empty();
	OpenList->ClearHeap();
	ClosedList.Clear();

	// ���� ��� ����
	TSharedPtr<FJPSNode> StartNode = MakeShared<FJPSNode>();

	// ������ġ ��� ���� ������ ������
	StartNode->Set(nullptr, JPSCoord(Start.Key, Start.Value), EndPos, 8);

	// ���� ��带 ����
	OpenList->Insert(StartNode);
	// Ž���Ϸ�� ��忡 ���� ��带 �߰�
	ClosedList.SetAt(StartNode->Pos.X, StartNode->Pos.Y, true);

	while (OpenList->GetCount())
	{
		// ���¸���Ʈ���� ���� �켱������ ���� ��� �˻� ����
		TSharedPtr<FJPSNode> CurrNode = OpenList->PopMin();
		// �����̿��� �ڿ��̿��� ������ �߰�
		int32 Directions = GetForcedNeighbours(CurrNode->Pos, CurrNode->CardinalDir) | GetNaturalNeighbours(CurrNode->CardinalDir);

		for (int32 Dir = 0; Dir < 8; Dir++)
		{
			// �ʿ��� ���� �˻�
			if ((1 << Dir) & Directions)
			{
				// �ش� �������� ��������Ʈ Ž��
				JPSCoord JumpPoint = Jump(CurrNode->Pos, Dir);
				// ��������Ʈ�� �����Ѵٸ�
				if (!JumpPoint.IsEmpty())
				{
					// ����
					if (JumpPoint == EndPos)
					{
						PathResults.Insert(EndPos, 0);

						// ���� ��带 �������� �������� ���󰡸鼭 ��������Ʈ ����� ����
						TSharedPtr<FJPSNode> TraceNode = CurrNode;
						int32 CurDir = GetCoordinateDir(EndPos, CurrNode->Pos);
						while (TraceNode.IsValid())
						{
							int32 NextDir = 0;
							// �θ� �ִٸ� ������->�θ� ������ ��ǥ�� ����
							if (TraceNode->Parent.IsValid())
							{
								NextDir = GetCoordinateDir(TraceNode->Pos, TraceNode->Parent->Pos);
							}
							// ���� ���� ����� ���� ���� ������ �ٸ��ٸ� ��Ͽ� �߰�
							if (CurDir != NextDir)
							{
								PathResults.Insert(TraceNode->Pos, 0);
							}
							// ���� �θ�� ����
							TraceNode = TraceNode->Parent;
							// ���� ����
							CurDir = NextDir;
						}

						// ��� �ܼ�ȭ
						PullingString(PathResults);
						// 3D��ǥȭ
						for (int32 Node = 0; Node < PathResults.Num(); Node++)
						{
							OutResultPos.Add(FieldCollision->GetNodeLocation(PathResults[Node].X, PathResults[Node].Y));
						}

						return true;
					}

					TSharedPtr<FJPSNode> NewNode = MakeShared<FJPSNode>();

					// ��������Ʈ ��带 ����
					NewNode->Set(CurrNode, JumpPoint, EndPos, Dir);

					// ó�� Ž���� ��ǥ��� ���� ���� ���
					if (!ClosedList.IsSet(JumpPoint.X, JumpPoint.Y))
					{
						// ���� ���ο� ���Ҹ� ���
						OpenList->Insert(NewNode);
						// Ž���� ��ǥ�� ���
						ClosedList.SetAt(JumpPoint.X, JumpPoint.Y, true);
					}
					else
					{
						// �̹� Ž���� ��ǥ��� �켱������ ����
						OpenList->InsertSmaller(NewNode);
					}
				}
			}
		}
	}

	return false;
}

TPair<int32, int32> UJPSPath::GetNorthEndPointReOpenBB(int32 InX, int32 InY)
{
	if (InX < 0 || InX >= GridWidth)
	{
		return TPair<int32, int32>(-1, -1);
	}

	if (FieldCollision->IsCollision(InX, InY))
	{
		// ���� ��ġ�� �̵� �Ұ��� �����̱⶧���� ������ �̵� ������ ���� �ΰ����� ��´�
		int32 OpenPos = FieldCollision->GetOpenValue(InX, InY, false, false);
		return TPair<int32, int32>(OpenPos, OpenPos);
	}
	else
	{
		// ���������� ������ ��ġ�� ���� ���������� �����ݴϴ�.
		// ������ �浹������ ã�´�
		int32 ClosePos = FieldCollision->GetCloseValue(InX, InY, false, false);
		// �浹������ �������� �浹���� ���Ŀ� ������ ���� ������ ã�´�
		int32 OpenPos = FieldCollision->GetOpenValue(InX, ClosePos, false, false);
		// ���� ����� ���������� �浹���� ������ ���� ������ ã�´�
		return TPair<int32, int32>(ClosePos + 1, OpenPos);
	}
}

TPair<int32, int32> UJPSPath::GetSouthEndPointReOpenBB(int32 InX, int32 InY)
{
	if (InX < 0 || InX >= GridWidth)
		return TPair<int32, int32>(GridHeight, GridHeight);

	if (FieldCollision->IsCollision(InX, InY))
	{
		int32 OpenPos = FieldCollision->GetOpenValue(InX, InY, false, true);
		return TPair<int32, int32>(OpenPos, OpenPos);
	}
	else
	{
		int32 ClosePos = FieldCollision->GetCloseValue(InX, InY, false, true);
		int32 OpenPos = FieldCollision->GetOpenValue(InX, ClosePos, false, true);
		return TPair<int32, int32>(ClosePos - 1, OpenPos);
	}
}

TPair<int32, int32> UJPSPath::GetEastEndPointReOpenBB(int32 InX, int32 InY)
{
	if (InY < 0 || InY >= GridHeight)
		return TPair<int32, int32>(GridWidth, GridWidth);

	if (FieldCollision->IsCollision(InX, InY))
	{
		int32 OpenPos = FieldCollision->GetOpenValue(InX, InY, true, true);
		return TPair<int32, int32>(OpenPos, OpenPos);
	}
	else
	{
		int32 ClosePos = FieldCollision->GetCloseValue(InX, InY, true, true);
		int32 OpenPos = FieldCollision->GetOpenValue(ClosePos, InY, true, true);
		return TPair<int32, int32>(ClosePos - 1, OpenPos);
	}
}

TPair<int32, int32> UJPSPath::GetWestEndPointReOpenBB(int32 InX, int32 InY)
{
	if (InY < 0 || InY >= GridHeight)
		return TPair<int32, int32>(-1, -1);

	if (FieldCollision->IsCollision(InX, InY))
	{
		int32 OpenPos = FieldCollision->GetOpenValue(InX, InY, true, false);
		return TPair<int32, int32>(OpenPos, OpenPos);
	}
	else
	{
		int32 ClosePos = FieldCollision->GetCloseValue(InX, InY, true, false);
		int32 OpenPos = FieldCollision->GetOpenValue(ClosePos, InY, true, false);
		return TPair<int32, int32>(ClosePos + 1, OpenPos);
	}
}

JPSCoord UJPSPath::NextCoordinate(const JPSCoord& InCoord, const int32 InDir)
{
	// �� �ϵ� �� ���� �� ���� �� �ϼ� ����
	static int32 dirMov[] = { 0,-1,1,-1,1,0,1,1,0,1,-1,1,-1,0,-1,-1,0,0 };
	return JPSCoord(InCoord.X + dirMov[InDir * 2], InCoord.Y + dirMov[InDir * 2 + 1]);
}

int32 UJPSPath::GetCoordinateDir(const JPSCoord& InSCoord, const JPSCoord& InDirCoord)
{
	int32 Dirs = 0;
	// ����
	if (InSCoord.X > InDirCoord.X)
	{
		Dirs += 1 << 4;
	}
	// ����
	else if (InSCoord.X < InDirCoord.X)
	{
		Dirs += 1 << 2;
	}
	// ����
	if (InSCoord.Y > InDirCoord.Y)
	{
		Dirs += 1 << 3;
	}
	// ����
	else if (InSCoord.Y < InDirCoord.Y)
	{
		Dirs += 1 << 1;
	}
	return Dirs;
}

int32 UJPSPath::GetForcedNeighbours(const JPSCoord& InCoord, const int32 InDir)
{
	//	7	0	1
	//	6		2
	//	5	4	3
	if (InDir > 7)
	{
		return 0;
	}

	int32 Dirs = 0;
#define ENTERABLE(n) IsPassable ( NextCoordinate (InCoord, (InDir + (n)) % 8))

	// �밢���ΰ��
	if (DirIsDiagonal(InDir))
	{
		// ex) Dir == 3 // ENT(1), ENT(0)
		if (!Implies(ENTERABLE(6), ENTERABLE(5)))
			Dirs = AddDirectionToSet(Dirs, (InDir + 6) % 8);
		// ex) Dir == 3 // ENT(5), ENT(6)
		if (!Implies(ENTERABLE(2), ENTERABLE(3)))
			Dirs = AddDirectionToSet(Dirs, (InDir + 2) % 8);
	}
	else
	{
		// ex) Dir == 2 // ENT(1), ENT(0)
		if (!Implies(ENTERABLE(7), ENTERABLE(6)))
			Dirs = AddDirectionToSet(Dirs, (InDir + 7) % 8);
		// ex) Dir == 2 // ENT(3), ENT(4)
		if (!Implies(ENTERABLE(1), ENTERABLE(2)))
			Dirs = AddDirectionToSet(Dirs, (InDir + 1) % 8);
	}

#undef ENTERABLE	

	return Dirs;
}


int32 UJPSPath::GetNaturalNeighbours(const int32 InDir)
{
	if (InDir == NODIRECTION)
		return 255;

	int32 Dirs = 0;
	// ���� ���� �߰�
	Dirs = AddDirectionToSet(Dirs, InDir);
	// ���� ������ �밢���̸� �ش� ������ ���� ���⵵ �߰�
	if (DirIsDiagonal(InDir))
	{
		Dirs = AddDirectionToSet(Dirs, (InDir + 1) % 8);
		Dirs = AddDirectionToSet(Dirs, (InDir + 7) % 8);
	}
	return Dirs;
}
bool UJPSPath::GetJumpPoint(JPSCoord InSCoord, const char direction, JPSCoord& OutJumpPoint)
{
	// ������ǥ
	InSCoord = NextCoordinate(InSCoord, direction);

	if (!IsPassable(InSCoord))
	{
		return false;
	}

	bool Ret = false;
	TPair<int32, int32> Up, Center, Down;
	// ������⿡ ����
	switch (direction)
	{

	case 0://North
		// ������ġ ���� ����, �߾�, ������ Ÿ���� �������� ���� Ž��

		//		0	0	0	0	0	0
		//		0	0	0	0	0	0
		//		0	0	0	0	0	0
		//		0	0	u	c	d	0
		//
		Up = GetNorthEndPointReOpenBB(InSCoord.X - 1, InSCoord.Y);
		Center = GetNorthEndPointReOpenBB(InSCoord.X, InSCoord.Y);
		Down = GetNorthEndPointReOpenBB(InSCoord.X + 1, InSCoord.Y);

		// x��ǥ ��ġ, �������� y��ǥ�� ������⿡ ����, �������� y��ǥ���� ������ġ���� Ž���� ���� �� openpoint�� �������� ������ġ ���̿� ����
		if (InSCoord.X == EndPos.X && InSCoord.Y >= EndPos.Y && Center.Key <= EndPos.Y)
		{
			OutJumpPoint = EndPos;
			return true;
		}
		// Down������ Ž������ �̵� ������ ������ �ִ�, 
		// Down������ Ž���������� ���� ���� ������ �����Ѵ�, 
		// ���� ��ġ���� �̵� ������ �������� Down ��ġ���� �̵� ������ ������ �� ���
		// Down ��ġ������ ���� ���� ������ ���� ��ġ�� �̵� ������ �������� �ּ� 2ĭ �ڿ� �ִ�
		// Ȥ��
		// Down���� ���������� �̵� ������ ������ ���� ���������� �������
		// Down ��ġ������ ���� ���� ������ ���� ��ġ�� �̵� ������ �������� �ּ� 2ĭ �ڿ� �ִ�
		// �� �� ������ �ϳ��� �����ϸ� Down ��ġ������ �������� �Ʒ��κ��� �����̿����� ���Ѵ�
		// ���� �����̿� �����̶� ����
		if (Down.Key != -1 && ((Down.Value > -1 && Down.Key > Center.Key && Down.Value + 2 > Center.Key) || (Down.Key == Down.Value && Down.Key + 2 > Center.Key)))
		{
			OutJumpPoint = JPSCoord(InSCoord.X, Down.Value + 1);
			Ret = true;
		}
		if (Up.Key != -1 && ((Up.Value > -1 && Up.Key > Center.Key && Up.Value + 2 > Center.Key) || (Up.Key == Up.Value && Up.Key + 2 > Center.Key)))
		{
			// ������ ������ ��������Ʈ���� ���� ������ ����� ���� ����
			OutJumpPoint = JPSCoord(InSCoord.X, Ret ? FMath::Max(OutJumpPoint.Y, Up.Value + 1) : Up.Value + 1);
			return true;
		}
		return Ret;
	case 2://EAST
		Up = GetEastEndPointReOpenBB(InSCoord.X, InSCoord.Y - 1);
		Center = GetEastEndPointReOpenBB(InSCoord.X, InSCoord.Y);
		Down = GetEastEndPointReOpenBB(InSCoord.X, InSCoord.Y + 1);

		if (InSCoord.Y == EndPos.Y && InSCoord.X <= EndPos.X && Center.Key >= EndPos.X)
		{
			OutJumpPoint = EndPos;
			return true;
		}

		if (Down.Key != GridWidth && ((Down.Value < GridWidth && Down.Key < Center.Key && Down.Value - 2 < Center.Key) || (Down.Key == Down.Value && Down.Key - 2 < Center.Key)))
		{
			OutJumpPoint = JPSCoord(Down.Value - 1, InSCoord.Y);
			Ret = true;
		}
		if (Up.Key != GridWidth && ((Up.Value < GridWidth && Up.Key < Center.Key && Up.Value - 2 < Center.Key) || (Up.Key == Up.Value && Up.Key - 2 < Center.Key)))
		{
			OutJumpPoint = JPSCoord(Ret ? FMath::Min(OutJumpPoint.X, Up.Value - 1) : Up.Value - 1, InSCoord.Y);
			return true;
		}
		return Ret;
	case 4://SOUTH
		Up = GetSouthEndPointReOpenBB(InSCoord.X - 1, InSCoord.Y);
		Center = GetSouthEndPointReOpenBB(InSCoord.X, InSCoord.Y);
		Down = GetSouthEndPointReOpenBB(InSCoord.X + 1, InSCoord.Y);

		if (InSCoord.X == EndPos.X && InSCoord.Y <= EndPos.Y && Center.Key >= EndPos.Y)
		{
			OutJumpPoint = EndPos;
			return true;
		}
		if (Down.Key != GridHeight && ((Down.Value < GridHeight && Down.Key < Center.Key && Down.Value - 2 < Center.Key) || (Down.Key == Down.Value && Down.Key - 2 < Center.Key)))
		{
			OutJumpPoint = JPSCoord(InSCoord.X, Down.Value - 1);
			Ret = true;
		}
		if (Up.Key != GridHeight && ((Up.Value < GridHeight && Up.Key < Center.Key && Up.Value - 2 < Center.Key) || (Up.Key == Up.Value && Up.Key - 2 < Center.Key)))
		{
			OutJumpPoint = JPSCoord(InSCoord.X, Ret ? FMath::Min(OutJumpPoint.Y, Up.Value - 1) : Up.Value - 1);
			return true;
		}
		return Ret;
	case 6://WEST
		Up = GetWestEndPointReOpenBB(InSCoord.X, InSCoord.Y - 1);
		Center = GetWestEndPointReOpenBB(InSCoord.X, InSCoord.Y);
		Down = GetWestEndPointReOpenBB(InSCoord.X, InSCoord.Y + 1);

		if (InSCoord.Y == EndPos.Y && InSCoord.X >= EndPos.X && Center.Key <= EndPos.X)
		{
			OutJumpPoint = EndPos;
			return true;
		}
		if (Down.Key != -1 && ((Down.Value > -1 && Down.Key > Center.Key && Down.Value + 2 > Center.Key) || (Down.Key == Down.Value && Down.Key + 2 > Center.Key)))
		{
			OutJumpPoint = JPSCoord(Down.Value + 1, InSCoord.Y);
			Ret = true;
		}
		if (Up.Key != -1 && ((Up.Value > -1 && Up.Key > Center.Key && Up.Value + 2 > Center.Key) || (Up.Key == Up.Value && Up.Key + 2 > Center.Key)))
		{
			OutJumpPoint = JPSCoord(Ret ? FMath::Max(OutJumpPoint.X, Up.Value + 1) : Up.Value + 1, InSCoord.Y);
			return true;
		}
		return Ret;
	}
	return false;
}

JPSCoord UJPSPath::Jump(const JPSCoord& InCoord, const char InDir)
{
	// ���� ��ǥ
	JPSCoord NextCoord = NextCoordinate(InCoord, InDir);
	// �밢��
	bool IsDiag = (bool)DirIsDiagonal(InDir);
	JPSCoord Offset(0, 0);
	Offset = NextCoordinate(Offset, InDir);

	while (1)
	{
		if (!IsPassable(NextCoord))
		{
			return JPSCoord(-1, -1);
		}
		// ���� ��ǥ�� �����̿��� �����ų� ��������� ��������Ʈ�� ����
		if (GetForcedNeighbours(NextCoord, InDir) || EndPos == NextCoord)
		{
			return NextCoord;
		}
		// �밢��
		if (IsDiag)
		{
			JPSCoord NewJumpPoint(-1, -1);
			// ��������Ʈ �˻� �밢�� ���� �翷 (�ϼ����̸� ���� ����)
			// �˻��� �翷������ ��������Ʈ ������ �����ϴ� ���� �ִٸ� ���� ������ ��������Ʈ�� ����
			if (GetJumpPoint(NextCoord, (InDir + 7) % 8, NewJumpPoint))
			{
				return NextCoord;
			}
			if (GetJumpPoint(NextCoord, (InDir + 1) % 8, NewJumpPoint))
			{
				return NextCoord;
			}
		}
		else
		{
			// �����˻��� �ش� �������� �� �˻� ��������Ʈ�� �ִٸ� �ش� ���� ��ȯ
			JPSCoord NewJumpPoint(-1, -1);
			GetJumpPoint(NextCoord, InDir, NewJumpPoint);
			return NewJumpPoint;
		}
		// ��������Ʈ�� ã�� ���ߴٸ� ����������� ������ǥ ������Ʈ
		NextCoord.Add(Offset);
	}
	return JPSCoord(-1, -1);
}

bool UJPSPath::PullingString(TArray<JPSCoord>& InResultNodes)
{
	// ��������� ��θ� ����ȭ
	if (InResultNodes.Num() <= 2)
	{
		return false;
	}

	int32 BaseNodeIndex = 0;
	int32 PrevNodeIndex = 1;
	int32 CurrNodeIndex = 2;

	while (CurrNodeIndex < InResultNodes.Num())
	{
		// ��� ���
		const JPSCoord& BaseNode = InResultNodes[BaseNodeIndex];
		// ��� ���� ���
		const JPSCoord& CurrNode = InResultNodes[CurrNodeIndex];

		// ���� ��忡�� ���� ������ �������� ���� �������� Ȯ��
		if (IsStraightPassable(BaseNode.X, BaseNode.Y, CurrNode.X, CurrNode.Y))
		{
			// �߰� ��� ����
			InResultNodes.RemoveAt(PrevNodeIndex);
		}
		else
		{
			// ������ �� ���ٸ� ���� ��带 ���� ���� ����
			BaseNodeIndex = PrevNodeIndex;
			PrevNodeIndex = CurrNodeIndex;
			CurrNodeIndex++;
		}
	}

	return InResultNodes.Num() > 0;
}

bool UJPSPath::IsStraightPassable(int32 InFromX, int32 InFromY, int32 InToX, int32 InToY)
{
	// ��ǥ ����
	float DiffX = (float)(InToX - InFromX);
	float DiffY = (float)(InToY - InFromY);
	// �̵����� �� ū��
	float Step = FMath::Max(FMath::Abs(DiffX), FMath::Abs(DiffY));

	// �̵� ����
	float Dx = DiffX / Step;
	float Dy = DiffY / Step;

	// �̵�����ŭ ���
	for (int i = 0; i <= FMath::CeilToInt(Step); i++)
	{
		// ������ġ�κ��� ���������� �밢���� �׾ �ش� ��ġ�� �ش��ϴ� ������ �˻�
		JPSCoord Pos(InFromX + int32(Dx * (float)i), InFromY + int32(Dy * (float)i));
		// �ش� ��ǥ�� �� ��ǥ ���̿� �����ϴ��� �˻�
		if (FMath::IsWithinInclusive(Pos.X, FMath::Min(InFromX, InToX), FMath::Max(InFromX, InToX)) == false)
		{
			continue;
		}

		if (FMath::IsWithinInclusive(Pos.Y, FMath::Min(InFromY, InToY), FMath::Max(InFromY, InToY)) == false)
		{
			continue;
		}

		// �ش� SPOT�� Passable �˻�
		// �� �� �ϳ��� �̵� �Ұ����� ��尡 ������ false
		if (IsPassable(Pos) == false)
		{
			return false;
		}
	}

	return true;
}
