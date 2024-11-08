# JPSSample

이 프로젝트는 JPS-B 길찾기 알고리즘을 언리얼 엔진에 구현한 프로젝트입니다


## 프로젝트 소개

기존 언리얼 엔진의 Navmesh를 이용한 경로탐색이 아닌 JPS 알고리즘과 BitScan을 사용하여 경로탐색을 시도합니다.

1. Nvmesh와 교차검증하여 특정 구역을 2D그리드화 합니다. 
2. BitScan을 접목해 가속화된 JPS 알고리즘을 사용해서 경로를 탐색합니다.
3. 2D그리드의 좌표를 3D좌표로 전환하여 경로탐색을 종료합니다.

## 설치 및 실행 방법
1. 프로젝트를 클론합니다.
2. JPSSample.uproject파일을 우클릭하여 Generate Visual Studio project files를 누릅니다.
3. JPSSample.sln 파일을 실행하여 Development Editor옵션을 선택 후 프로젝트를 컴파일 해줍니다. 
4. UE5 프로젝트를 실행합니다.

## 키 설명
1. Q키 : 경로탐색
