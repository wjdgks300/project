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
이 error를 해결하려면 vcpkg를 설치하라고 했다.  하지만 visual studio 2019에서는 안된다는 말도 있었고, vcpkg로 설치후 해보니까 잘안되서 ubuntu 환경에서 다시 하기로 했다 

-----------
### 2일 차 
ubuntu 에서 마저 실패,,, 일단 이거 잠시 놔두고 python으로 구현 된것으로 일단 실행해보고 한줄 한줄 코멘트 넣어서 올려보자..  


________________
### 3일 차 
github에 올라와있는 python 코드를 발견하고 그대로 실행해보고, 한줄 한줄 분석하여 보았다. 
중간에 x,y 크기를 비교하여 일정크기 이상 안되도록 하는 부분이 잘 이해가 가지 않았다.
특히 ans=[int (((x_m - x)/(y_m - y))*( y_m+Step - y) + x),y_m+Step] 이 부분은 어떻게 계산되었는지 잘 모르겠다. 

그리고 screen.get_at((x,y))에서 값이 (0,0,0,255) 처럼 4개의 값이 나오는데 get_at함수는 좌표의 색깔을 나타내는 함수다.
어떻게 4개의 값이 나오는지 모르겠다. 무슨 색깔이 저렇게 표현되는 것인지 확신이 없다. 

#### 일단 정리해보면 다음과 같다. 

1. 무작위로 샘플을 뽑는다. 
2. 뽑은 샘플과 가장 가까운 정점 v를 찾는다. 
3. 선택한 v를 어떤 계산을 통하여 새로운 v로 계산하고
4. 가장 가까운 정점 v와 새로운 x, y 좌표 사이에 장애물이 있는지 확인 한다. 
5. 장애물이 없으면 v를 업데이트 한다.
6. goal을 찾으면 parent함수를 이용하여 처음 시작점(-1,-1) 까지 찾는다.

실행을 하면 다음과 같은 사진이 나오게 된다. 

### 장애물 없음 
![장애물 없음](https://user-images.githubusercontent.com/63197363/180750999-584e4550-61ef-47f6-82fb-4b9e3cba0d38.png)




### 장애물 있음 
![장애물1](https://user-images.githubusercontent.com/63197363/180751249-5d3a358a-7df3-407e-bc38-7120acdc5711.png)



다음 목표 
-> https://jdj2261.github.io/review/2021/10/07/rrt-star-review.html 여기에 rrt 에 관한 설명이 있으니 이거 보고 참고하여 공부를 하고 모르는 것들 알아내기! 


--------------------
[int (((x_m - x)/(y_m - y))*( y_m+Step - y) + x),y_m+Step]  이 부분에서 step은 한번에 이동할 거리이고, 이 좌표는 이동했을 때의 좌표인것 같다. 
이유는 이전에 이러한 코드가 나온다. 

for u in range(y_m+1, y+1):                                
                    x_cur = int (((x_m - x)/(y_m - y))*( u - y) + x)
                    y_cur = u
                    if screen.get_at((x_cur,y_cur)) == (0,0,0,255):         
                        good=False
                        break

여기 부분은 장애물이 있는지 확인하는 코드인데, x_cur 과 y_cur을 계산하는 부분을 보면 위의 [int (((x_m - x)/(y_m - y))*( y_m+Step - y) + x),y_m+Step] 와 매우 유사하다. x_cur, y_cur에서 step 만큼만 이동시켜 준것으로 보인다. 

그리고 step size가 커지면 목표지점을 빨리 찾고, 작은 step size이면 좀 더 최적의 길을 찾는다라는 설명을 봐서 step size는 한번에 이동하는 거리가 맞다고 생각한다. 

## RRT* 알고리즘 

RRT* 알고리즘은 RRT알고리즘의 성능을 Optomality 관점에서 개선한 경로 생성 방법론이다. RRT 알고리즘의 결과는 최적의 해답이 아니기 때문에 이를 개선한 것이 RRT*이다. 
### RRT* 알고리즘만의 차별점 
- 비용함수 도입
- new state의 neighbor들을 Optimal path로 Rewiring 작업 추가 

![jksaa-27-2-1-g1](https://user-images.githubusercontent.com/63197363/181547688-146cd3a2-7a84-4535-8425-2e630ba09f25.jpeg)


RRT* 알고리즘은 RRT 알고리즘과 처음은 유사하다. (a,b)
먼저 처음에 랜덤 샘플링을 하고, 새로운 노드를 가장 가까운 노드와 연결한다. 
그 다음 과정은 최적의 부모를 찾는 과정이다.(c,d)  Xinit로 부터 Xnew까지의 비용을 고려하여 비용이 더 적은 path를 다시 구하는 과정이다. 

그 후 rewiring 작업을 시작한다. (e,f) rewiring 작업은 neighbor 노드들을 탐색하여 주변 이웃 노드들의 path를 한번 더 최적화 하는 과정이다. 
-> 이웃 노드들에 대해 비용 관점에서 최적화를 진행하여 Optimality를 향상 시킨다. 


