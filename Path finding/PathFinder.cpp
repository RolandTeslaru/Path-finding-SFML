//
//  PathFinder.cpp
//  Path finding
//
//  Created by Roland Teslaru on 13.06.2022.
//
#include "PathFinder.hpp"
#include "array"
#include "queue"
#include "vector"
#include "stack"

int CheckedprimalSq = 0;

inline bool operator < (const Square& lhs, const Square& rhs)
{//We need to overload "<" to put our struct into a set
    return lhs.fCost < rhs.fCost;
}

AStar::AStar( Square *PointA ,  Square *PointB ,  Matrix *_Matrix)
{
    StartSq = PointA;
    FinishSq = PointB;
    MainMatrix = _Matrix;
}

AStar::~AStar(){}

void AStar::init(Square MainSquare[100][100])
{
    for(int i = 1 ; i<= MainMatrix -> Mat_I ; i++)
    {
        for(int j = 1 ; j<= MainMatrix -> Mat_J ;j++)
        {
            MainSquare[i][j].gCost = INFINITY;
            MainSquare[i][j].hCost = INFINITY;
            MainSquare[i][j].fCost = INFINITY;
            MainSquare[i][j].ParentPoz = Vector2i(-1,-1);
            MainSquare[i][j].poz = Vector2i(j,i);
            
        }
    }
    int i = StartSq -> poz.y;
    int j = StartSq -> poz.x;
    MainSquare[i][j].fCost = 0.0;
    MainSquare[i][j].gCost = 0.0;
    MainSquare[i][j].hCost = 0.0;
}

void AStar::tracepath(Square MainSquare[100][100])
{
    int i = FinishSq -> poz.y;
    int j = FinishSq -> poz.x;
    stack<Square> path;
    vector<Square> usablePath;

    while(MainSquare[i][j].ParentPoz != Vector2i(j,i) &&
          MainSquare[i][j].poz != Vector2i(-1,-1))
    {
        path.push(MainSquare[i][j]);
        int tempX = MainSquare[i][j].ParentPoz.x;
        int tempY = MainSquare[i][j].ParentPoz.y;
        i = tempY;
        j = tempX;
    }
    path.push(MainSquare[i][j]);
    while(!path.empty())
    {
        Square top = path.top();
        MainSquare[top.poz.y][top.poz.x].sqColor = Color::Green;
        path.pop();
        usablePath.emplace_back(top);
    }
}

bool AStar::isAvailable(Square testSq)
{
    if( MainMatrix -> Mat[testSq.poz.y][testSq.poz.x] == 1 )
        return false;
    else
        return true;
}

bool AStar::isDestionation(Square currentSq)
{
    if(currentSq . poz.y == FinishSq -> poz.y &&
       currentSq . poz.x == FinishSq -> poz.x)
        return true;
    else
        return false;
}

bool AStar::isSqValid(Square testSq )
{
    if( testSq.poz.y <= MainMatrix -> Mat_I &&
        testSq.poz.x <= MainMatrix -> Mat_J &&
        testSq.poz.y >= 1 &&
        testSq.poz.x >= 1)
        return true;
    else
        return false;
}

double AStar::herusitic(Square currentSq )
{
    return sqrt(pow((StartSq -> poz.y - FinishSq -> poz.y), 2.0  ) + pow((StartSq -> poz.x - FinishSq -> poz.x) , 2.0));
}
 
void AStar::Alg(Square MainSquare[100][100])
{
    if(isDestionation(*StartSq))
    {
        cout << "you are at the destination" << endl;
        return;
    }
    
    bool closedlist[101][101];
    
    for(int i = 1 ; i<= MainMatrix -> Mat_I ; i++)
        for(int j = 1 ; j<= MainMatrix -> Mat_J ;j++)
            closedlist[i][j] = false;

    int i = StartSq -> poz.y;
    int j = StartSq -> poz.x;


    MainSquare[i][j].ParentPoz = Vector2i(j , i);
    
    vector<Square> openlist;
    openlist.emplace_back(MainSquare[i][j]);
    bool pathFound = false;
    openlist.begin();
    
    while (!openlist.empty())
    {
        Square currentSq;
        do
        {
            float temp = INFINITY;
            vector<Square>::iterator itSquare;
            vector<Square>::iterator it = openlist.begin();
            for(it = openlist.begin() ; it != openlist.end() ; it = next(it))
            {
                Square n = *it;
                if(n.fCost < temp)
                {
                    temp = n.fCost;
                    itSquare = it;
                }
            }
            currentSq = *itSquare;
            openlist.erase(itSquare);
            
        }while(isSqValid(currentSq) == false);
        
        i = currentSq.poz.y;
        j = currentSq.poz.x;
        closedlist[i][j] = true;
        
        for(int addY = -1 ; addY <= 1 ; addY ++)
        {
            for(int addX = -1 ; addX <= 1 ; addX ++)
            {
                double gNew , fNew , hNew;
                if(isSqValid(MainSquare[i+addY][j+addX] )&&
                    isAvailable(MainSquare[i+addY][j+addX]))
                {
                    if(isDestionation(MainSquare[i+addY][j+addX]))
                    {
                        MainSquare[i+addY][j+addX].ParentPoz = Vector2i(j , i);
                        pathFound = true;
                        cout << "PAth has been found " << endl;
                        return;
                    }
                    else if(closedlist[i + addY][j + addX] == false)
                    {
                        gNew = currentSq.gCost + 1.0;
                        hNew = herusitic(MainSquare[i+addY][i+addX]);
                        fNew = gNew + hNew;
                        
                        if(MainSquare[i + addY][j + addX].fCost == INFINITY ||
                           MainSquare[i + addY][j + addX].fCost > fNew)
                        {
                            MainSquare[i + addY][j + addX].fCost = fNew;
                            MainSquare[i + addY][j + addX].hCost = hNew;
                            MainSquare[i + addY][j + addX].gCost = gNew;
                            MainSquare[i + addY][j + addX].ParentPoz = Vector2i(j,i);
                            
                            openlist.emplace_back(MainSquare[i+addY][j+addX]);
                        }
                    }
                }
            }
        }
    }
    if(!pathFound)
        cout << "Destination could not be found :(" << endl;
    return ;
        
}
