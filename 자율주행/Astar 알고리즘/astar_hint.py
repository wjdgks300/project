import math
import re
import copy

# grid를 출력하는 함수
def print_grid(grid):
    dx = len(grid)
    dy = len(grid[0])
    for x in range(dx):
        for y in range(dy):
            print(grid[x][y], end=' ')
        print()
    return


def read_list(line):
    r = re.compile('[ \t\n\r]+')        # 정규표현의 처리를 실행하기 위해서 정규표현 패턴의 메소드를 사용한다.
    #print(r)
    s_list = r.split(line)
    #print(s_list)
    n = len(s_list) - 1
    i_list = []
    for i in range(n):
        if s_list[i] == '':
            break
        i_list.append(s_list[i])            #한 줄로 나누기 위해서 i_list로 뽑아놓은거 같다.
    #print(i_list)
    return i_list

#map 읽기
def read_grid(fname):
    f = open(fname, 'r')
    line = f.readline() # read dimension info 한줄 한줄 읽어 온다.
    list = read_list(line)                      #맨 위의 27 / 27 을 호출한다.
    #print(list)

    line = f.readline()          # skip empty line   [27][27] 다음인 비어있는 줄을 뛰어 넘는다.
    grid = []
    while True:
        line = f.readline()
        if line == '':
            break
        #print(line)
        list = read_list(line)                  # 한 줄씩 받아서 저장하고, grid에 추가한다.
        grid.append(list)

    dx = len(grid)
    dy = len(grid[0])
    print('grid['+str(dx)+']['+str(dy)+'] read!')
    f.close()
    return grid

def never_visited(v):
    if v == '1':
        return False
    elif v == 'X':
        return False
    elif v == 'C':
        return False
    else:
        return True

def search(grid, grid2, bx, by):
    F = abs(goalx - bx) + abs(goaly - by)
    count = 0
    open_list.append([bx, by, '', F]) ### extended to include the F value

    while len(open_list) > 0:                           # Q에 남은게 없을때 까지 반복
        point = get_next_pose(open_list)                # 앞에꺼 받아온다.

        for item in open_list:                 # 오픈 리스트 노드들중에 가장 최소 비용을 지닌 노드를 찾는다.
            if item[3] < point[3]:
                point = item

        open_list.remove(point)                         # 받아온거 제거
        closed_list.append(point)                       # 닫힌 리스트에 해당 노드 추가

        x = point[0]
        y = point[1]
        print('visiting %d,%d F = %d' % (x, y, point[3]))       # 방문한 좌표 출력
        #print_grid(grid2); input('press any key')
        if grid[x][y] == '9':                                   # goal에 도달하면 종료
            print('found %d,%d' % (x,y))
            return True
        else:
            if never_visited(grid2[x][y-1]):                    # 남쪽으로 이동

                F = calc_fn(start, goal, [x, y-1])
                grid2[x][y - 1] = 'X'
                open_list.append([x, y-1, point, F])
            if never_visited(grid2[x-1][y]):                    # 서쪽으로 이동

                F = calc_fn(start, goal, [x-1, y])
                grid2[x - 1][y] = 'X'
                open_list.append([x-1, y, point, F])
            if never_visited(grid2[x][y+1]):                    # 동쪽으로 이동

                F = calc_fn(start, goal, [x, y+1])
                grid2[x][y + 1] = 'X'
                open_list.append([x, y+1, point, F])
            if never_visited(grid2[x+1][y]):                    # 북쪽으로 이동

                F = calc_fn(start, goal, [x+1, y])
                grid2[x + 1][y] = 'X'
                open_list.append([x+1, y, point, F])
            grid2[x][y] = 'C'
        count = count + 1
        print("count = ",count)
    return False

def mark_path(list, grid):              # 마지막 closed list 를 통해 가장 짫은 경로를 '.'로 표시한다.
    point=list[-1]
    while True:
        x = point[0]
        y = point[1]
        point = point[2]
        grid[x][y] = '.'
        if point == '':
            break
    return

def calc_fn(startPoint, goalPoint, currPoint):
    gn = int(math.sqrt(math.pow(currPoint[0] - goalPoint[0], 2)))
    hn = int(math.sqrt(math.pow(currPoint[1] - goalPoint[1], 2)))
    return gn + hn

def get_next_pose(openlist):
    point = openlist[0] ### openlist 원소 중에서 F 값이 최소인 원소를 찾아서 리턴하라.
    return point

grid = read_grid('map2.txt') # map0.txt, map1.txt, map2.txt, map3.txt
grid2 = copy.deepcopy(grid)                 # grdi2 에 grid 를 deepcopy한다.
lines = [i for i in grid if '8' in i]          # 시작점 8 찾기
print("lines =" ,lines)
bx = grid.index(lines[0])                       # 시작점의 x 좌표
by = grid[bx].index('8')                        # 시작점의 y 좌표
# print("bx , by ", bx, by)

goalpoint = [j for j in grid if '9' in j]      # goal 은 9
#print("goalpoint =" ,goalpoint)
goalx = grid.index(goalpoint[0])                # 도착점의 x 좌표
goaly = grid[goalx].index('9')                  # 도착점의 y 좌표

goal = [goalx, goaly]
start = [bx, by]

#print_grid(grid)
closed_list = []
open_list = []
if search(grid, grid2, bx, by):
    print('grid map')
    print_grid(grid); print()
    mark_path(closed_list, grid)
    print('grid map with path marked')
    print_grid(grid)

print(closed_list)