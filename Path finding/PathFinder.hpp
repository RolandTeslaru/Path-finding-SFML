//
//  PathFinder.hpp
//  Path finding
//
//  Created by Roland Teslaru on 13.06.2022.
//

#ifndef PathFinder_hpp
#define PathFinder_hpp

#include <stdio.h>
#include "GlobalValue.h"
#include "GlobalLIbs.h"
#include "stack"
#include "Matrix.hpp"
#include "Render.hpp"


class AStar
{
private:
    
public:
    Square **StartSq;
    Square **FinishSq;

    Matrix **MainMatrix;
    stack<Square> OpenList;
    stack<Square> ClosedList;
    
    AStar( Square *PointA ,  Square *PointB ,  Matrix *_Matrix);
    
    ~AStar();
    
    bool foundDest = false;
        
    bool isSqValid(Square *currentSq );
    bool finished(Square *currentSq );
    
    void init();
    
    Square currentNode;
    
    double herusitic(Square *currentSq , string type);
    double EUdistance(Square *currentSq );
    double ManhattenDistance(Square *currentSq);
    double DiagonalDistance(Square *currentSq);
    
    void Alg(Square MainSquare[100][100]);
};

class Node
{
    struct sqNode
    {
        bool Obstacle;
    };
    
    
    
    friend class Matrix;
    friend class Square;
};

#endif /* PathFinder_hpp */
