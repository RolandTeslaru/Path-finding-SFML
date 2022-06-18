//
//  input.hpp
//  Path finding
//
//  Created by Roland Teslaru on 17.06.2022.
//

#ifndef input_hpp
#define input_hpp

#include <stdio.h>
#include "GlobalLIbs.h"
#include "Matrix.hpp"

class input
{
private:
//    bool PointA = false;
//    bool PointB = false;
    bool AlgStat = false;
public:
    void reset();
    
    void setPointA(bool status);
    void setPointB(bool status);
    void setBarrier(sf::Vector2i poz);
    
    void deleteBarrier(sf::Vector2i poz);
    
    void getinput(sf::Mouse MainMouse , Matrix *MainMatrix , Square MainSqaure[][100] , Square *PointA , Square *PointB , int i , int j);
    
    void startAlg();
};



#endif /* input_hpp */
