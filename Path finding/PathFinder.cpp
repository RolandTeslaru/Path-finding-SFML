//
//  PathFinder.cpp
//  Path finding
//
//  Created by Roland Teslaru on 13.06.2022.
//

#include "PathFinder.hpp"

AStar::AStar( Square *PointA ,  Square *PointB ,  Matrix *_Matrix)
{
    StartSq = PointA;
    FinishSq = PointB;
    MainMatrix = _Matrix;
}

AStar::~AStar(){}

void AStar::init()
{
    OpenList.push(*StartSq);
    StartSq->fCost =0;
}

bool AStar::finished(Square *currentSq )
{
    if(currentSq -> poz == FinishSq -> poz)
        return true;
    else
        return false;
}

bool AStar::isSqValid(Square *currentSq )
{
    if( MainMatrix -> Mat[currentSq -> poz.y][currentSq -> poz.x] == 0 )
        return true;
    else
        return false;
}

double AStar::ManhattenDistance(Square *currentSq)
{
    double h = abs(currentSq -> poz.x - FinishSq->poz.x) +
               abs(currentSq -> poz.y - FinishSq->poz.y);
    return h;
}

double AStar::EUdistance(Square *currentSq)
{
    int c1 = currentSq -> poz.y - FinishSq->poz.y;
    int c2 = currentSq -> poz.x - FinishSq->poz.x;
    double c3 = sqrt(c1*c1 + c2*c2);
    return c3;
}

double AStar::DiagonalDistance(Square *currentSq)
{

    double dx = abs(currentSq -> poz.x - FinishSq->poz.x);
    double dy = abs(currentSq -> poz.y - FinishSq->poz.y);
    double D = 1;
    double D2 = sqrt(2);
    double h = D * (dx + dy) + (D2 - 2 * D) * min(dx , dy);
    
    return h;
}

double AStar::herusitic(Square *currentSq , string type)
{
    return EUdistance(currentSq );
}
 
void AStar::Alg(Square MainSquare[100][100])
{
    Square CurrentSq;
}
