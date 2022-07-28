#Python Source Code

#Library Imports
import pygame
from random import randint as ri
pygame.init()
import time

#GAME Parameters
screen = pygame.display.set_mode([500, 550])
GAME_x = 20
GAME_y = 40
GAME_width = 440
GAME_height = 400
GAME_border = 3
WHITE=(255,255,255)
BLUE=(0,0,255)
BLACK=(0,0,0)
RED=(255,0,0)
GREEN=(0,255,0)
custom_color_1=(10,145,80)
screen.fill(WHITE)
INT_MAX = 100000000000000
#Class Definitions
class Button:
    def __init__ (self, colour, x, y, width, height):
        self.colour = colour
        self.x = x
        self.y = y
        self.width = width
        self.height = height
    def create(self,screen):
        pygame.draw.rect(screen, self.colour, [self.x, self.y,self.width ,self.height])


#Function Definition : Point inside Game ?
def point_inside_game(x,y):
    if x>GAME_x+GAME_border and x<GAME_x + GAME_width - GAME_border:
                if y>GAME_y+GAME_border and y < GAME_y + GAME_height - GAME_border:
                    return(True)
    return(False)


#Function Definition : Random Point Generator inside Game
def random_point():
    x_random = ri(GAME_x+GAME_border , GAME_x + GAME_width - GAME_border - 1)
    y_random = ri(GAME_y+GAME_border , GAME_y + GAME_height - GAME_border - 1 )
    return((x_random, y_random))


#Function Definition : Point inside given Rectangle ?
def point_inside_rec(xr,yr,wr,hr,x,y):
    if x> xr and x < xr + wr:
                if y > yr and y < yr + hr:
                    return(True)
    return(False)


#Function Definition : Point to Point Distance 점과 점사이의 거리
def p2p_dist(p1,p2):
    x1,y1=p1
    x2,y2=p2
    return ( ( (x1-x2)**2 + (y1-y2)**2 )**0.5 )


#Function Definition : Text on Button
def ClickText():
    font = pygame.font.Font('freesansbold.ttf', 12)
    text = font.render('CLICK HERE', True, WHITE)
    textRect = text.get_rect()
    textRect.center = (75, 495)
    screen.blit(text, textRect)


#Function Definition : Description Text
def DesText(s,x=315,y=485):
    pygame.draw.rect(screen,WHITE,(125,470,500,30))
    font = pygame.font.SysFont('segoeuisemilight', 15)
    text = font.render('%s'%(s), True, BLACK)
    textRect = text.get_rect()
    #textRect.center = (255, 460)
    textRect.center = (x, y)
    screen.blit(text, textRect)


#Function Definition :RRT Algorithm
def RRT(x,y,parent):
    if (x,y) not in parent and screen.get_at((x,y)) != (0,0,0,255): #get_at((x,y)) 는 픽셀 색상값을 얻는 것 -> (0,0,0,255) 아닐때 까지(목표 색깔)
        x_m,y_m=-1,-1
        cur_min=INT_MAX
        # print(screen.get_at((x,y)))
        for v in parent:
            if p2p_dist(v,(x,y))<cur_min:                   # 점과 점사이의 거리가 cur_min 보다 작을 때 x_m, y_m 이 v가 됨
                x_m,y_m=v                                   # 이전 점의 거리들 중에서 제일 가까운 점 v를 반환한다.
                cur_min =  p2p_dist(v,(x,y))

        good = True
        ans=[]
        if abs(x_m - x)<abs(y_m-y):                                         #y의 값의 변화가 클 때
            if y_m<y:                                                       # v의 y값보다 현재 y가 큰 값을 가지면
                for u in range(y_m+1, y+1):                                 # v의 y에서 y까지 반복한다.
                    x_cur = int (((x_m - x)/(y_m - y))*( u - y) + x)
                    y_cur = u
                    if screen.get_at((x_cur,y_cur)) == (0,0,0,255):         # 만약 v에서 y까지 가는데 장애물이 있으면 good에 false를 반환한다.
                        good=False
                        break
                if good:
                    ans=[int (((x_m - x)/(y_m - y))*( y_m+Step - y) + x),y_m+Step]      # 장애물이 없으면 ans에 다음과 같은 값을 반환한다.
            else:
                for u in range(y, y_m):
                    x_cur = int(((x_m - x)/(y_m - y))*( u - y) + x)
                    y_cur = u
                    if screen.get_at((x_cur,y_cur)) == (0,0,0,255):
                        good=False
                        break
                if good:
                    ans=[int (((x_m - x)/(y_m - y))*( y_m-Step - y) + x),y_m-Step]

        else:
            if x_m<x:                                                      #x의 값이 변화가 클 때
                for u in range(x_m + 1, x+1):
                    x_cur = u
                    y_cur = int( ((y_m-y)/(x_m-x))*(u-x) + y )
                    if screen.get_at((x_cur,y_cur)) == (0,0,0,255):
                        good=False
                        break
                if good:
                    ans=[x_m+Step,int( ((y_m-y)/(x_m-x))*(x_m+Step-x) + y ) ]
            else:
                for u in range(x , x_m):
                    x_cur = u
                    y_cur = int( ((y_m-y)/(x_m-x))*(u-x) + y )
                    if screen.get_at((x_cur,y_cur)) == (0,0,0,255):
                        good=False
                        break
                if good:
                    ans=[x_m-Step,int( ((y_m-y)/(x_m-x))*(x_m-Step-x) + y ) ]
        return(good,x_m,y_m,ans)
    return(False,-1,-1,[])

running = True
#Button for Game
pygame.draw.rect(screen,BLACK,(GAME_x,GAME_y,GAME_width,GAME_height),GAME_border)
B1 = Button(BLACK, 25, 470, 100, 50)
B1.create(screen)
OBS=dict()

#Number of forward Steps towards random sampled point
Step = 10
#Start stores a single point [Starting point- RED Point]
Start=[]

#End stores a set of destination point [Destination point- Green Point]
#Multiple points allowed to make the point appear bigger, and fast discovery,
#due to huge number of pixels in this game
End=set()


#parent stores the graph
parent=dict()       # dictionary 함수를 사용하여 parent 함수를 만듬
level=1
ClickText()
DesText("Instruction :",y=460)
DesText("Draw the Obstacles, then CLICK BLACK Button")
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
        if running==False:
            break
        m = pygame.mouse.get_pressed()
        x,y = pygame.mouse.get_pos()

        if m[0]==1:
            if point_inside_rec(B1.x,B1.y, B1.width, B1.height,x,y):
                    #print("BUTTON", level)
                    if level==1 and Start==[]:
                        level+=1
                        B1.colour=RED
                        DesText("Draw the Starting point, then CLICK RED Button")
                    elif level==2 and Start:
                        level+=1
                        B1.colour=GREEN
                        DesText("Draw the Destination point, then CLICK GREEN Button")
                    elif level==3 and End!=set():
                        level+=1
                        B1.colour=BLUE
                        DesText("Path is being explored using RRT Algorithm")
                    B1.create(screen)
                    ClickText()
                    continue
            elif level==1:
                if point_inside_game(x,y):
                    #print("OBSTABLE ",x,y)
                    OBS[(x,y)]=1
                    pygame.draw.circle(screen, BLACK, (x, y), 10)
            elif level == 2 and Start==[]:
                if point_inside_game(x,y):
                    #print("START ",x,y)
                    Start=(x,y)
                    pygame.draw.circle(screen, RED, (x, y), 5)
            elif level == 3:
                if point_inside_game(x,y):
                    #print("END ",x,y)
                    End.add((x,y))
                    pygame.draw.circle(screen, GREEN, (x, y), 10)

        if level>=4:
            running = False
            break
    pygame.display.update()

running = True
parent[Start]=(-1,-1)               # parent 에  (-1,-1) 을 처음에 넣어줌
Trace=[]                            # 선들을 추적해서 올라가기 위한 함수
Timer =  time.time()
while(running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    x,y =random_point()
    if (time.time() - Timer) > 5:
        Step=5
    good,x_m,y_m,ans=RRT(x,y,parent)        # 랜덤 좌표 (x,y)를 RRT 알고리즘에 전달, parent 전달

    if good and ans:                            # 점과 점사이에 장애물이 없을 때
        x_cur = ans[0]                          # 현재 이동한 x 점
        y_cur = ans[1]                          # 현재 이동한 y 점
        if screen.get_at((x_cur,y_cur)) != (0,0,0,255) and (x_cur,y_cur) not in parent:     # 이전에 방문안했고, (0,0,0,255) 아니면
            parent[(x_cur,y_cur)]=(x_m,y_m)                                                 # parent x_cur, y_cur에 부모 좌표인 x_m, y_m을 넣는다.
            if screen.get_at((x_cur,y_cur)) == (0, 255, 0, 255):                            # 목적지 도착했으면 이전에 방문했던 좌표를 따라 trace
                Trace=(x_cur,y_cur)
                print("End", x_cur, y_cur, x_m,y_m)
                running = False
            pygame.draw.line(screen, BLUE, (x_cur,y_cur), (x_m,y_m), 2)                     # 이동할 좌표를 파란색으로 이어준다. 자식 (x_cur,y_cur) <- 부모(x_m,y_m)
    pygame.display.update()

running = True
#This loop gets the route back to Start point
while(Trace and running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            break
    while(Trace!=Start):                                                                # 목적지까지 도달한 점들을 초록색으로 칠한다.
        x,y = parent[Trace]                                                             # parent(x,y)를 통하여 가장 가까운 x,y값을 불러오고
        pygame.draw.line(screen, GREEN, (x,y), Trace, 2)                                # x, y부터 trace 까지 초록색으로 이어준다.
        Trace=(x,y)                                                                     # x, y 는 cur_x , cur_y 로 바꿔주고 start가 될때 까지 반복한다.
        #print("Trace",x,y)
    DesText("Green Colored Path is the Required Path")
    pygame.display.update()

#Quit the Game
pygame.quit()
