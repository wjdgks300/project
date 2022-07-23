# RRT 알고리즘 (Rapidly exploring Random Tree)

RRT 알고리즘은 무작위 샘플링을 사용하여 고차원의 구성 공간을 탐색하는 경로계획 알고리즘이다. 
중앙 시작지점에서 경로의 후보들이 확장되기 시작하여 탐색공간 전체에 대한 경로 후보들을 확보하고, 경로 후보들 중 최적의 경로를 선택하는 것이 핵심이다. 

----------- 
RRT 알고리즘 구현 방법은 다음과 같다.

![The-basic-RRT-construction-algorithm](https://user-images.githubusercontent.com/63197363/180209896-ea4ea0f6-c3d2-40b0-b6ff-c02f5243bb19.png)


1. 시작지점을 설정한다.
2. qrand( 경로 후보 확장을 위해 뿌리는 샘플링 포인트로 확장하는점)를 뿌려서 확장하고 기존의 점과 가장 가까운 점을 연결하게 되는데, 여기서 기존 트리에서 가장 가까운 점이 바로 Qnear 이다. 
3. qrand와 연결이 되면서 새로운 포인트는 qnew이다. 
4. qnew를 기분으로 다시 qrand를 뿌리면서 계속적으로 확장해 나간다. 


RRT path planning을 하기위해서는 map이 있어야 하고, 현재 위치를 알아야 하며, 목표지점을 알아야한다. 그리고 로봇이 한번에 갈 수있는 최대 크기를 정해야 한다. 


구현은 다음 사이트를 참고 하였다. https://github.com/markusbuchholz/Path_Planning_RRT_algorithm
------------

### 1일 차 
위의 코드대로 visual studio 2019에 실행하려고 하니까 matplotlibcpp 에러가 떴다. 
이 error를 해결하려면 vcpkg를 설치하라고 했다. 
