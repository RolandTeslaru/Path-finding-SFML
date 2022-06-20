//
//  PathFinder.cpp
//  Path finding
//
//  Created by Roland Teslaru on 13.06.2022.
//

#include "PathFinder.hpp"
#include "array"
#include "queue"
#include "set"
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
        for(int j = 1 ; j<= MainMatrix -> Mat_J ; j++)
        {
            MainSquare[i][j].poz.y = i;
            MainSquare[i][j].poz.x = j;
            MainSquare[i][j].fCost = -1;
            MainSquare[i][j].hCost = -1;
            MainSquare[i][j].gCost = -1;
            MainSquare[i][j].ParentPoz.y = -1;
            MainSquare[i][j].ParentPoz.x = -1;
        }
    }
    MainSquare[StartSq -> poz.y][StartSq -> poz.x].fCost = 0.0;
    MainSquare[StartSq -> poz.y][StartSq -> poz.x].hCost = 0.0;
    MainSquare[StartSq -> poz.y][StartSq -> poz.x].gCost = 0.0;
    
    foundDest = false;
}

void AStar::tracepath(Square MainSquare[100][100])
{
    stack<Square> Path;
    int row = FinishSq -> poz.y;
    int col = FinishSq -> poz.x;
    Square nextnode;
    nextnode.poz = MainSquare[row][col].ParentPoz;
    
    do
    {
        Path.push(nextnode);
        nextnode.poz = MainSquare[row][col].ParentPoz;
        
        row = nextnode.poz.y;
        col = nextnode.poz.x;
    }while(MainSquare[row][col].ParentPoz != nextnode.poz);
    
    Path.emplace(MainSquare[row][col]);
    
    while(!Path.empty())
    {
        Square p = Path.top();
        Path.pop();
        if(p.poz.y > 0 && p.poz.x > 0)
            MainSquare[p.poz.y][p.poz.x].sqColor = Color::Green;
    }
}

bool AStar::finished(Square currentSq )
{
    if(currentSq . poz == FinishSq->poz)
        return true;
    else
        return false;
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

double AStar::ManhattenDistance(Square currentSq)
{
    double h = abs(currentSq . poz.x - FinishSq->poz.x) +
    abs(currentSq . poz.y - FinishSq->poz.y);
    return h;
}

double AStar::EUdistance(Square currentSq)
{
    int c1 = currentSq . poz.y - FinishSq->poz.y;
    int c2 = currentSq . poz.x - FinishSq->poz.x;
    double c3 = sqrt(c1*c1 + c2*c2);
    return c3;
}

double AStar::DiagonalDistance(Square currentSq)
{

    double dx = abs(currentSq . poz.x - FinishSq->poz.x);
    double dy = abs(currentSq . poz.y - FinishSq->poz.y);
    double D = 1;
    double D2 = sqrt(2);
    double h = D * (dx + dy) + (D2 - 2 * D) * min(dx , dy);
    
    return h;
}

double AStar::herusitic(Square currentSq )
{
    return sqrt(pow((StartSq -> poz.y - FinishSq -> poz.y), 2.0  ) + pow((StartSq -> poz.x - FinishSq -> poz.x) , 2.0));
//    return EUdistance(currentSq );
}
 
void AStar::Alg(Square MainSquare[100][100])
{
        
//    if(isSqValid(StartSq) == false)
//    {
//        cout << "Start Sq   " <<   StartSq -> poz.y << "   " << StartSq -> poz.x << endl;
//        cout << " Source is invalid " << endl;
//        return ;
//    }
    
    stack<Square> OpenList;
    
    int i = StartSq->poz.y ;
    int j = StartSq->poz.x ;
    
    bool ClosedList[MainMatrix -> Mat_I][MainMatrix -> Mat_J];
    memset(ClosedList, false, sizeof(ClosedList));
    
    OpenList.emplace(*StartSq);
        
    while(!OpenList.empty())
    {
        Square currentSq;
        const Square& p = OpenList.top();
        
        currentSq.poz = p.poz;
        i = p.poz.y;
        j = p.poz.x;
        OpenList.pop();
        
        ClosedList[i][j] = true;
        
//        if(isDestionation(p))
//        {
//            MainSquare[p.poz.y][p.poz.x].ParentPoz = p.poz;
//            cout << "Early cheack destination cell has been found" << endl;
//            return ;
//         }
//
        //Check every neighbour
        for(int addX = -1 ; addX <= 1 ; addX ++)
        {
            for(int addY = -1 ; addY <= 1 ; addY++)
            {
                Square neighbour = MainSquare[i+addY][j+addX];
                if(isSqValid(neighbour))
                {
//                    neighbour.poz = p.poz;
//                    neighbour.hCost = herusitic(neighbour);
//                    neighbour.gCost = neighbour.gCost + 1.0;
//                    neighbour.fCost = neighbour.hCost + neighbour.gCost;
                    
                    if(isDestionation(neighbour))
                    {
                        MainSquare[neighbour.poz.y][neighbour.poz.x].ParentPoz = Vector2i(j , i);
                        cout << "Destination cell has been found " << endl;
                        return;
                    }
                    else if(!ClosedList[neighbour.poz.y][neighbour.poz.x] &&
                             isAvailable(neighbour))
                    {
                        double gNew , hNew , fNew;
                        gNew = MainSquare[i][j].gCost + 1.0;
                        hNew = herusitic(neighbour);
                        fNew = gNew + hNew;

                        if(MainSquare[neighbour.poz.y][neighbour.poz.x].fCost ==  -1 ||
                           MainSquare[neighbour.poz.y][neighbour.poz.x].fCost >fNew)
                        {
                            neighbour.fCost = fNew;
                            OpenList.emplace(neighbour);

                            MainSquare[neighbour.poz.y][neighbour.poz.x].gCost = gNew;
                            MainSquare[neighbour.poz.y][neighbour.poz.x].hCost = hNew;
                            MainSquare[neighbour.poz.y][neighbour.poz.x].fCost = fNew;
                            MainSquare[neighbour.poz.y][neighbour.poz.x].ParentPoz = Vector2i(j,i);
                        }
                    }
                }
            }
        }
    }
    if(foundDest == false)
        cout << "Could not find destination :( " << endl;
    return ;
        
}
