//
//  Path.cpp
//  Path finding
//
//  Created by Roland Teslaru on 05.06.2022.
//
#include "Path.hpp"
#include "GlobalValue.h"
#include "GlobalLIbs.h"
#include "Render.hpp"
#include "Matrix.hpp"
#include "input.hpp"

Square MainSquare[100][100];
Matrix MainMatrix;

int mainpath()
{
    MainMatrix.Mat_I = 30;
    MainMatrix.Mat_J = 30;
    
    initwindow();
    render(MainSquare, &MainMatrix);
    
    return 0;
}

