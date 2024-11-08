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

	//범위 체크
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

	// 시작 노드 생성
	TSharedPtr<FJPSNode> StartNode = MakeShared<FJPSNode>();

	// 시작위치 노드 세팅 방향은 전방향
	StartNode->Set(nullptr, JPSCoord(Start.Key, Start.Value), EndPos, 8);

	// 시작 노드를 오픈
	OpenList->Insert(StartNode);
	// 탐색완료된 노드에 시작 노드를 추가
	ClosedList.SetAt(StartNode->Pos.X, StartNode->Pos.Y, true);

	while (OpenList->GetCount())
	{
		// 오픈리스트에서 가장 우선순위가 높은 노드 검사 시작
		TSharedPtr<FJPSNode> CurrNode = OpenList->PopMin();
		// 강제이웃과 자연이웃의 방향을 추가
		int32 Directions = GetForcedNeighbours(CurrNode->Pos, CurrNode->CardinalDir) | GetNaturalNeighbours(CurrNode->CardinalDir);

		for (int32 Dir = 0; Dir < 8; Dir++)
		{
			// 필요한 방향 검사
			if ((1 << Dir) & Directions)
			{
				// 해당 방향으로 점프포인트 탐색
				JPSCoord JumpPoint = Jump(CurrNode->Pos, Dir);
				// 점프포인트가 존재한다면
				if (!JumpPoint.IsEmpty())
				{
					// 도착
					if (JumpPoint == EndPos)
					{
						PathResults.Insert(EndPos, 0);

						// 현재 노드를 기준으로 역순으로 따라가면서 점프포인트 목록을 구성
						TSharedPtr<FJPSNode> TraceNode = CurrNode;
						int32 CurDir = GetCoordinateDir(EndPos, CurrNode->Pos);
						while (TraceNode.IsValid())
						{
							int32 NextDir = 0;
							// 부모가 있다면 현재노드->부모 방향의 좌표를 구함
							if (TraceNode->Parent.IsValid())
							{
								NextDir = GetCoordinateDir(TraceNode->Pos, TraceNode->Parent->Pos);
							}
							// 현재 진행 방향과 새로 구한 방향이 다르다면 목록에 추가
							if (CurDir != NextDir)
							{
								PathResults.Insert(TraceNode->Pos, 0);
							}
							// 다음 부모로 변경
							TraceNode = TraceNode->Parent;
							// 방향 갱신
							CurDir = NextDir;
						}

						// 경로 단순화
						PullingString(PathResults);
						// 3D좌표화
						for (int32 Node = 0; Node < PathResults.Num(); Node++)
						{
							OutResultPos.Add(FieldCollision->GetNodeLocation(PathResults[Node].X, PathResults[Node].Y));
						}

						return true;
					}

					TSharedPtr<FJPSNode> NewNode = MakeShared<FJPSNode>();

					// 점프포인트 노드를 생성
					NewNode->Set(CurrNode, JumpPoint, EndPos, Dir);

					// 처음 탐색된 좌표라면 힙에 새로 등록
					if (!ClosedList.IsSet(JumpPoint.X, JumpPoint.Y))
					{
						// 힙에 새로운 원소를 등록
						OpenList->Insert(NewNode);
						// 탐색된 좌표로 등록
						ClosedList.SetAt(JumpPoint.X, JumpPoint.Y, true);
					}
					else
					{
						// 이미 탐색된 좌표라면 우선순위를 갱신
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
		// 현재 위치가 이동 불가능 지역이기때문에 다음에 이동 가능한 영역 두가지를 담는다
		int32 OpenPos = FieldCollision->GetOpenValue(InX, InY, false, false);
		return TPair<int32, int32>(OpenPos, OpenPos);
	}
	else
	{
		// 오픈지점의 마지막 위치와 다음 오픈지점을 돌려줍니다.
		// 북쪽의 충돌지점을 찾는다
		int32 ClosePos = FieldCollision->GetCloseValue(InX, InY, false, false);
		// 충돌지점을 기준으로 충돌지점 이후에 나오는 오픈 지점을 찾는다
		int32 OpenPos = FieldCollision->GetOpenValue(InX, ClosePos, false, false);
		// 가장 가까운 오픈지점과 충돌지점 이후의 오픈 지점을 찾는다
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
	// 북 북동 동 남동 남 남서 서 북서 정지
	static int32 dirMov[] = { 0,-1,1,-1,1,0,1,1,0,1,-1,1,-1,0,-1,-1,0,0 };
	return JPSCoord(InCoord.X + dirMov[InDir * 2], InCoord.Y + dirMov[InDir * 2 + 1]);
}

int32 UJPSPath::GetCoordinateDir(const JPSCoord& InSCoord, const JPSCoord& InDirCoord)
{
	int32 Dirs = 0;
	// 서쪽
	if (InSCoord.X > InDirCoord.X)
	{
		Dirs += 1 << 4;
	}
	// 동쪽
	else if (InSCoord.X < InDirCoord.X)
	{
		Dirs += 1 << 2;
	}
	// 북쪽
	if (InSCoord.Y > InDirCoord.Y)
	{
		Dirs += 1 << 3;
	}
	// 남쪽
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

	// 대각선인경우
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
	// 현재 방향 추가
	Dirs = AddDirectionToSet(Dirs, InDir);
	// 현재 방향이 대각선이면 해당 방향의 수직 방향도 추가
	if (DirIsDiagonal(InDir))
	{
		Dirs = AddDirectionToSet(Dirs, (InDir + 1) % 8);
		Dirs = AddDirectionToSet(Dirs, (InDir + 7) % 8);
	}
	return Dirs;
}
bool UJPSPath::GetJumpPoint(JPSCoord InSCoord, const char direction, JPSCoord& OutJumpPoint)
{
	// 다음좌표
	InSCoord = NextCoordinate(InSCoord, direction);

	if (!IsPassable(InSCoord))
	{
		return false;
	}

	bool Ret = false;
	TPair<int32, int32> Up, Center, Down;
	// 진행방향에 따라
	switch (direction)
	{

	case 0://North
		// 현재위치 기준 왼쪽, 중앙, 오른쪽 타일을 기준으로 북쪽 탐색

		//		0	0	0	0	0	0
		//		0	0	0	0	0	0
		//		0	0	0	0	0	0
		//		0	0	u	c	d	0
		//
		Up = GetNorthEndPointReOpenBB(InSCoord.X - 1, InSCoord.Y);
		Center = GetNorthEndPointReOpenBB(InSCoord.X, InSCoord.Y);
		Down = GetNorthEndPointReOpenBB(InSCoord.X + 1, InSCoord.Y);

		// x좌표 일치, 도달점의 y좌표가 진행방향에 있음, 도달점의 y좌표보다 현재위치에서 탐색된 가장 먼 openpoint가 도달점과 현재위치 사이에 있음
		if (InSCoord.X == EndPos.X && InSCoord.Y >= EndPos.Y && Center.Key <= EndPos.Y)
		{
			OutJumpPoint = EndPos;
			return true;
		}
		// Down방향의 탐색에서 이동 가능한 영역이 있다, 
		// Down방향의 탐색지점에서 다음 오픈 지점이 존재한다, 
		// 현재 위치에서 이동 가능한 영역보다 Down 위치에서 이동 가능한 영역이 더 길다
		// Down 위치에서의 다음 오픈 지점이 현재 위치의 이동 가능한 영역보다 최소 2칸 뒤에 있다
		// 혹은
		// Down에서 마지막으로 이동 가능한 영역과 다음 오픈지점이 같을경우
		// Down 위치에서의 다음 오픈 지점이 현재 위치의 이동 가능한 영역보다 최소 2칸 뒤에 있다
		// 위 두 조건중 하나라도 만족하면 Down 위치에서의 오픈지점 아랫부분을 강제이웃으로 정한다
		// 위의 강제이웃 조건이랑 부합
		if (Down.Key != -1 && ((Down.Value > -1 && Down.Key > Center.Key && Down.Value + 2 > Center.Key) || (Down.Key == Down.Value && Down.Key + 2 > Center.Key)))
		{
			OutJumpPoint = JPSCoord(InSCoord.X, Down.Value + 1);
			Ret = true;
		}
		if (Up.Key != -1 && ((Up.Value > -1 && Up.Key > Center.Key && Up.Value + 2 > Center.Key) || (Up.Key == Up.Value && Up.Key + 2 > Center.Key)))
		{
			// 이전에 설정된 점프포인트보다 현재 지점에 가까운 곳을 고른다
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
	// 다음 좌표
	JPSCoord NextCoord = NextCoordinate(InCoord, InDir);
	// 대각선
	bool IsDiag = (bool)DirIsDiagonal(InDir);
	JPSCoord Offset(0, 0);
	Offset = NextCoordinate(Offset, InDir);

	while (1)
	{
		if (!IsPassable(NextCoord))
		{
			return JPSCoord(-1, -1);
		}
		// 다음 좌표에 강제이웃이 있으거나 목적지라면 점프포인트로 지정
		if (GetForcedNeighbours(NextCoord, InDir) || EndPos == NextCoord)
		{
			return NextCoord;
		}
		// 대각선
		if (IsDiag)
		{
			JPSCoord NewJumpPoint(-1, -1);
			// 점프포인트 검사 대각선 기준 양옆 (북서쪽이면 북쪽 서쪽)
			// 검사후 양옆지점에 점프포인트 조건이 만족하는 곳이 있다면 현재 지점을 점프포인트로 지정
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
			// 직선검사라면 해당 방향으로 쭉 검사 점프포인트가 있다면 해당 지점 반환
			JPSCoord NewJumpPoint(-1, -1);
			GetJumpPoint(NextCoord, InDir, NewJumpPoint);
			return NewJumpPoint;
		}
		// 점프포인트를 찾지 못했다면 진행방향으로 다음좌표 업데이트
		NextCoord.Add(Offset);
	}
	return JPSCoord(-1, -1);
}

bool UJPSPath::PullingString(TArray<JPSCoord>& InResultNodes)
{
	// 지그재그인 경로를 직선화
	if (InResultNodes.Num() <= 2)
	{
		return false;
	}

	int32 BaseNodeIndex = 0;
	int32 PrevNodeIndex = 1;
	int32 CurrNodeIndex = 2;

	while (CurrNodeIndex < InResultNodes.Num())
	{
		// 출발 노드
		const JPSCoord& BaseNode = InResultNodes[BaseNodeIndex];
		// 출발 다음 노드
		const JPSCoord& CurrNode = InResultNodes[CurrNodeIndex];

		// 기준 노드에서 현재 노드까지 직선으로 도달 가능한지 확인
		if (IsStraightPassable(BaseNode.X, BaseNode.Y, CurrNode.X, CurrNode.Y))
		{
			// 중간 노드 제거
			InResultNodes.RemoveAt(PrevNodeIndex);
		}
		else
		{
			// 제거할 수 없다면 기준 노드를 이전 노드로 변경
			BaseNodeIndex = PrevNodeIndex;
			PrevNodeIndex = CurrNodeIndex;
			CurrNodeIndex++;
		}
	}

	return InResultNodes.Num() > 0;
}

bool UJPSPath::IsStraightPassable(int32 InFromX, int32 InFromY, int32 InToX, int32 InToY)
{
	// 좌표 차이
	float DiffX = (float)(InToX - InFromX);
	float DiffY = (float)(InToY - InFromY);
	// 이동량이 더 큰쪽
	float Step = FMath::Max(FMath::Abs(DiffX), FMath::Abs(DiffY));

	// 이동 비율
	float Dx = DiffX / Step;
	float Dy = DiffY / Step;

	// 이동량만큼 계산
	for (int i = 0; i <= FMath::CeilToInt(Step); i++)
	{
		// 시작위치로부터 목적지까지 대각선을 그어서 해당 위치에 해당하는 셀들을 검사
		JPSCoord Pos(InFromX + int32(Dx * (float)i), InFromY + int32(Dy * (float)i));
		// 해당 좌표가 두 좌표 사이에 존재하는지 검사
		if (FMath::IsWithinInclusive(Pos.X, FMath::Min(InFromX, InToX), FMath::Max(InFromX, InToX)) == false)
		{
			continue;
		}

		if (FMath::IsWithinInclusive(Pos.Y, FMath::Min(InFromY, InToY), FMath::Max(InFromY, InToY)) == false)
		{
			continue;
		}

		// 해당 SPOT이 Passable 검사
		// 그 중 하나라도 이동 불가능한 노드가 있으면 false
		if (IsPassable(Pos) == false)
		{
			return false;
		}
	}

	return true;
}
