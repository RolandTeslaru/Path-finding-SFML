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
    Square *StartSq;
    Square *FinishSq;

    Matrix *MainMatrix;
    set<Square> OpenList;
    
    AStar( Square *PointA ,  Square *PointB ,  Matrix *_Matrix);
    
    ~AStar();
    
    bool foundDest = false; 
        
    bool isSqValid(Square testSq );
    bool finished(Square currentSq );
    bool isDestionation(Square currentSq);
    bool isAvailable(Square testSq);
    
    void init(Square MainSquare[100][100]);
    
    void tracepath(Square MainSquare[100][100]);
    
    Square currentNode;
    
    Square createSq(Vector2i poz);
    
    double herusitic(Square currentSq);
    double EUdistance(Square currentSq );
    double ManhattenDistance(Square currentSq);
    double DiagonalDistance(Square currentSq);
    
    void Alg(Square MainSquare[100][100]);
};

#endif /* PathFinder_hpp */
