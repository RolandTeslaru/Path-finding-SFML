//
//  PathFinder.cpp
//  Path finding
//
//  Created by Roland Teslaru on 13.06.2022.
//

#include "PathFinder.hpp"

int FLT_MAX = 70;

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

void AStar::init()
{
    StartSq -> fCost = 0.0;
    StartSq -> gCost = 0.0;
    StartSq -> hCost = 0.0;
    StartSq -> ParentPoz.y = StartSq -> poz.y;
    StartSq -> ParentPoz.x = StartSq -> poz.x;
    
    OpenList.insert(*StartSq);
    
    foundDest = false;
}

Square AStar::createSq(Vector2i poz)
{
    Square newSq;
    newSq.poz = poz;
    return newSq;
}

bool AStar::finished(Square *currentSq )
{
    if(currentSq -> poz == FinishSq -> poz)
        return true;
    else
        return false;
}

bool AStar::isAvailable(Square *testSq)
{
    if( MainMatrix -> Mat[testSq->poz.y][testSq->poz.x] == 1 )
        return false;
    else
        return true;
}

bool AStar::isDestionation(Square *currentSq)
{
    if(currentSq -> poz.y == FinishSq -> poz.y &&
       currentSq -> poz.x == FinishSq -> poz.x)
        return true;
    else
        return false;
}

bool AStar::isSqValid(Square testSq )
{
    if( MainMatrix -> Mat[testSq.poz.y][testSq.poz.x] == 0 )
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
    bool ClosedList[MainMatrix -> Mat_I][MainMatrix -> Mat_J];
    memset(ClosedList, false, sizeof(ClosedList));
        
    if(isSqValid(*StartSq) == false)
    {
        cout << " Source is invalid " << endl;
        return ;
    }
    
    
    if(isAvailable(StartSq) == false ||
       isAvailable(StartSq) == false)
    {
        cout << "source at the destiantion is unblockec " << endl;
        return;
    }
    
    
    if(isDestionation(StartSq) == true)
    {
        cout << "We are alreadya t the destination " << endl;
        return ;
    }
    
    
    while(!OpenList.empty())
    {
        cout << " HELLO ? OPEN LIST . EXMPTY" << endl;
        Square currentSq = *OpenList.begin();
        
        Square P_currentSq = currentSq;
        OpenList.erase(OpenList.begin());

        int i = currentSq.poz.y;
        int j = currentSq.poz.x;
        ClosedList[i][j] = true;

        double gNew , hNew , fNew;

        P_currentSq.poz = Vector2i(j , i-1);
        
        
        
//        for(int i = 1 ; i<= MainMatrix -> Mat_I ; i++)
//        {
//            for(int j = 1 ; j<= MainMatrix -> Mat_J ; j++)
//            {
//                cout << MainMatrix -> Mat[i][j] << " ";
//            }
//            cout << endl;
//        }
//
        // ------- S U C C E S O R     1  (N) -------- //   I - 1
        if(isSqValid(MainSquare[i-1][j]))
        {
            if(isDestionation( &MainSquare[i-1][j]) == true)
            {
                
                MainSquare[i-1][j].ParentPoz.y = i;
                MainSquare[i-1][j].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i-1][j] == false &&
                    isAvailable(&MainSquare[i-1][j]) == true)
            {
                cout << " this should be true " << endl;
                gNew = MainSquare[i][j].gCost;
                hNew = EUdistance(&MainSquare[i-1][j]);
                fNew = gNew + hNew;
                
                if(MainSquare[i-1][j].fCost == FLT_MAX ||
                   MainSquare[i-1][j].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i-1][j]);
                    MainSquare[i-1][j].fCost = fNew;
                    MainSquare[i-1][j].gCost = gNew;
                    MainSquare[i-1][j].hCost = hNew;
                    MainSquare[i-1][j].ParentPoz.y = i;
                    MainSquare[i-1][j].ParentPoz.x = j;
                }
            }
        }
        
        // ------- S U C C E S O R     2 (S)-------- //    I + 1
        if(isSqValid(MainSquare[i+1][j]))
        {
            if(isDestionation( &MainSquare[i+1][j]) == true)
            {
                MainSquare[i+1][j].ParentPoz.y = i;
                MainSquare[i+1][j].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i+1][j] == false &&
                    isAvailable(&MainSquare[i+1][j]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.0;
                hNew = EUdistance(&MainSquare[i+1][j]);
                fNew = gNew + hNew;
                
                if(MainSquare[i+1][j].fCost == FLT_MAX ||
                   MainSquare[i+1][j].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i-1][j]);
                    MainSquare[i-1][j].fCost = fNew;
                    MainSquare[i-1][j].gCost = gNew;
                    MainSquare[i-1][j].hCost = hNew;
                    MainSquare[i-1][j].ParentPoz.y = i;
                    MainSquare[i-1][j].ParentPoz.x = j;
                }
            }
        }
        // ------- S U C C E S O R     3 (E)-------- //        J + 1
        if(isSqValid(MainSquare[i][j+1]))
        {
            if(isDestionation( &MainSquare[i][j+1]) == true)
            {
                MainSquare[i][j+1].ParentPoz.y = i;
                MainSquare[i][j+1].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i][j+1] == false &&
                    isAvailable(&MainSquare[i][j+1]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.0;
                hNew = EUdistance(&MainSquare[i][j+1]);
                fNew = gNew + hNew;
                
                if(MainSquare[i][j+1].fCost == FLT_MAX ||
                   MainSquare[i][j+1].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i][j+1]);
                    MainSquare[i][j+1].fCost = fNew;
                    MainSquare[i][j+1].gCost = gNew;
                    MainSquare[i][j+1].hCost = hNew;
                    MainSquare[i][j+1].ParentPoz.y = i;
                    MainSquare[i][j+1].ParentPoz.x = j;
                }
            }
        }
        // ------- S U C C E S O R     4 (W)-------- //     J - 1
        if(isSqValid(MainSquare[i][j-1]))
        {
            if(isDestionation( &MainSquare[i][j-1]) == true)
            {
                MainSquare[i][j-1].ParentPoz.y = i;
                MainSquare[i][j-1].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i][j-1] == false &&
                    isAvailable(&MainSquare[i][j-1]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.0;
                hNew = EUdistance(&MainSquare[i][j+1]);
                fNew = gNew + hNew;
                
                if(MainSquare[i][j-1].fCost == FLT_MAX ||
                   MainSquare[i][j-1].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i][j-1]);
                    MainSquare[i][j-1].fCost = fNew;
                    MainSquare[i][j-1].gCost = gNew;
                    MainSquare[i][j-1].hCost = hNew;
                    MainSquare[i][j-1].ParentPoz.y = i;
                    MainSquare[i][j-1].ParentPoz.x = j;
                }
            }
        }
        
        // ------- S U C C E S O R     5 ( N-E)-------- //        I - 1    J + 1
        if(isSqValid(MainSquare[i-1][j+1]))
        {
            if(isDestionation( &MainSquare[i-1][j+1]) == true)
            {
                MainSquare[i-1][j+1].ParentPoz.y = i;
                MainSquare[i-1][j+1].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i-1][j+1] == false &&
                    isAvailable(&MainSquare[i-1][j+1]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.414;
                hNew = EUdistance(&MainSquare[i-1][j+1]);
                fNew = gNew + hNew;
                
                if(MainSquare[i-1][j+1].fCost == FLT_MAX ||
                   MainSquare[i-1][j+1].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i][j+1]);
                    MainSquare[i-1][j+1].fCost = fNew;
                    MainSquare[i-1][j+1].gCost = gNew;
                    MainSquare[i-1][j+1].hCost = hNew;
                    MainSquare[i-1][j+1].ParentPoz.y = i;
                    MainSquare[i-1][j+1].ParentPoz.x = j;
                }
            }
        }
        // ------- S U C C E S O R     6 ( N-W)-------- //      I - 1    J - 1
        if(isSqValid(MainSquare[i-1][j-1]))
        {
            if(isDestionation( &MainSquare[i-1][j-1]) == true)
            {
                MainSquare[i-1][j-1].ParentPoz.y = i;
                MainSquare[i-1][j-1].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i-1][j-1] == false &&
                    isAvailable(&MainSquare[i-1][j-1]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.414;
                hNew = EUdistance(&MainSquare[i-1][j-1]);
                fNew = gNew + hNew;
                
                if(MainSquare[i-1][j-1].fCost == FLT_MAX ||
                   MainSquare[i-1][j-1].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i][j-1]);
                    MainSquare[i-1][j-1].fCost = fNew;
                    MainSquare[i-1][j-1].gCost = gNew;
                    MainSquare[i-1][j-1].hCost = hNew;
                    MainSquare[i-1][j-1].ParentPoz.y = i;
                    MainSquare[i-1][j-1].ParentPoz.x = j;
                }
            }
        }
        
        // ------- S U C C E S O R     7 ( S-E )-------- //       I + 1    J + 1
        if(isSqValid(MainSquare[i+1][j+1]))
        {
            if(isDestionation( &MainSquare[i+1][j+1]) == true)
            {
                MainSquare[i+1][j+1].ParentPoz.y = i;
                MainSquare[i+1][j+1].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i+1][j+1] == false &&
                    isAvailable(&MainSquare[i+1][j+1]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.414;
                hNew = EUdistance(&MainSquare[i+1][j+1]);
                fNew = gNew + hNew;
                
                if(MainSquare[i+1][j+1].fCost == FLT_MAX ||
                   MainSquare[i+1][j+1].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i+1][j+1]);
                    MainSquare[i+1][j+1].fCost = fNew;
                    MainSquare[i+1][j+1].gCost = gNew;
                    MainSquare[i+1][j+1].hCost = hNew;
                    MainSquare[i+1][j+1].ParentPoz.y = i;
                    MainSquare[i+1][j+1].ParentPoz.x = j;
                }
            }
        }
        
        // ------- S U C C E S O R     8 ( S-W )-------- //       I + 1    J - 1
        if(isSqValid(MainSquare[i+1][j-1]))
        {
            if(isDestionation( &MainSquare[i+1][j-1]) == true)
            {
                MainSquare[i+1][j-1].ParentPoz.y = i;
                MainSquare[i+1][j-1].ParentPoz.x = j;
                cout << "Destination found" << endl;
                foundDest = true;
                return ;
            }
            else if(ClosedList[i+1][j-1] == false &&
                    isAvailable(&MainSquare[i+1][j-1]) == true)
            {
                gNew = MainSquare[i][j].gCost + 1.414;
                hNew = EUdistance(&MainSquare[i+1][j-1]);
                fNew = gNew + hNew;
                
                if(MainSquare[i+1][j-1].fCost == FLT_MAX ||
                   MainSquare[i+1][j-1].fCost > fNew)
                {
                    OpenList.insert(MainSquare[i+1][j-1]);
                    MainSquare[i+1][j-1].fCost = fNew;
                    MainSquare[i+1][j-1].gCost = gNew;
                    MainSquare[i+1][j-1].hCost = hNew;
                    MainSquare[i+1][j-1].ParentPoz.y = i;
                    MainSquare[i+1][j-1].ParentPoz.x = j;
                }
            }
        }
    }
    if(foundDest == false)
        cout << "Could not find destination :( " << endl;
    return ;
}
