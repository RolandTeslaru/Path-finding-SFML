//
//  Matrix.hpp
//  Path finding
//
//  Created by Roland Teslaru on 05.06.2022.
//

#ifndef Matrix_hpp
#define Matrix_hpp

#include "GlobalValue.h"
#include "GlobalLIbs.h"

using namespace sf;
using namespace std;

class Square
{
private:
public:
    sf::Color sqColor = sf::Color::White;
    Vector2i poz ;    
    
    Vector2i ParentPoz;
    float gCost;    // -- Distance to the start node
    float hCost;    // -- Estimated distance to the goal
    float fCost;
    
    Square *parent;
    
};

class Matrix
{
private:
public:
    int Mat_I , Mat_J;
    int Mat[100][100] = {0};
};

#endif /* Matrix_hpp */
