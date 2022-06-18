//
//  Render.cpp
//  Path finding
//
//  Created by Roland Teslaru on 05.06.2022.
//
#include "Render.hpp"
#include "GlobalValue.h"
#include "Matrix.hpp"
#include "input.hpp"
using namespace sf;
using namespace std;
 
bool ready = false;
Vector2i mousePoz;

Square PointA;
Square PointB;

sf::RenderWindow window(VideoMode(WinX , WinY + 120) , "path finding");
sf::Font font;
sf::Font Menlo;
sf::Text TXTpoint;
sf::Text TXTCredits;

sf::Mouse MainMouse;

int inputmode;

void initwindow()
{
    PointA.poz = Vector2i(-1 , -1);
    PointB.poz = Vector2i(-1 , -1);
    
    window.clear(Color::Black);
    window.setFramerateLimit(60);
    
    if(!font.loadFromFile("Resources/font.ttf") ||
       !Menlo.loadFromFile("Resources/Menlo-Regular.ttf"))
    {
        cout << "Error loading text font " << endl;
        
        exit(EXIT_FAILURE);
    }
    TXTpoint.setFont(font);
    TXTpoint.setScale(1, 1);
    TXTpoint.setPosition(Vector2f(0, WinY ));

    TXTCredits.setFont(Menlo);
    TXTCredits.setScale(1, 1);
    TXTCredits.setPosition(Vector2f(0,WinY +99));
    TXTCredits.setString("by Teslaru Roland");
    TXTCredits.setFillColor(Color::White);
    TXTCredits.setCharacterSize(20);
}
int happening = 0;

void render(Square MainSquare[][100] , Matrix *MainMatrix)
{
    sf::RectangleShape block(Vector2f(WinX / MainMatrix -> Mat_J - 2 , WinY / MainMatrix -> Mat_I - 2 ));

    while(window.isOpen())
    {
        mousePoz = MainMouse.getPosition(window);
        
        window.clear(Color::Black);
        Event event;
        while(window.pollEvent(event))
        {
            if(event.type == event.Closed)
                window.close();
        }
        
        for(int i = 1 ; i <= MainMatrix -> Mat_I ; i++)
        {
            for(int j = 1 ; j <= MainMatrix -> Mat_J ; j++)
            {
                if(!ready)
                {
                    //  --------  I N P U T   ---------   //
                    
                    //-------- Identify square that overlaps with mouse
                    
                    if( mousePoz.x > WinX / MainMatrix -> Mat_I * (j-1) &&
                        mousePoz.x < WinX / MainMatrix -> Mat_I * (j) &&
                        mousePoz.y > WinY / MainMatrix -> Mat_I * (i-1) &&
                        mousePoz.y < WinX / MainMatrix -> Mat_I * (i) )
                    {
                        if(MainMouse.isButtonPressed(sf::Mouse::Left))
                        {
                            //------ Select starting square
                            if(PointA.poz == Vector2i(-1 , -1))
                            {
                                PointA.poz = Vector2i(i,j);
                                MainSquare[i][j].sqColor = Color::Blue;
                                MainMatrix -> Mat[i][j] = 2;
                            }
                            //------ Select Finishing Sqaure
                            else if(PointA.poz != Vector2i(-1 , -1) &&
                                    PointB.poz == Vector2i(-1 , -1))
                            {
                                PointB.poz = Vector2i(i,j);
                                MainSquare[i][j].sqColor = Color::Red;
                                MainMatrix -> Mat[i][j] = 3;
                            }
                            //------ Select barrier blocks
                            else if(PointA.poz != Vector2i(-1 , -1) &&
                                    PointB.poz != Vector2i(-1 , -1) &&
                                    MainMatrix -> Mat[i][j] == 0)
                            {
                                MainMatrix -> Mat[i][j] = 1;
                                MainSquare[i][j].sqColor = Color::Black;
                            }
                        }
                        
                        //----- Deselect barrier block
                        if(MainMouse.isButtonPressed(sf::Mouse::Right))
                        {
                            if(MainMatrix -> Mat[i][j] == 1)
                            {
                                MainMatrix -> Mat[i][j] = 0;
                                MainSquare[i][j].sqColor = Color::White;
                            }
                        }
                        
                        //------ Auto highlight square that overlaps with mouse
                        if(MainMatrix -> Mat[i][j] == 0)
                            MainSquare[i][j].sqColor = Color(199 , 200 , 200);
                        
                    }
                }
                
                block.setFillColor(MainSquare[i][j].sqColor);
                block.setPosition(WinX / MainMatrix -> Mat_J * (j - 1) , WinY / MainMatrix -> Mat_I * (i-1) );
                window.draw(block);
                
                // Revert highlited square
                if(MainSquare[i][j].sqColor == Color(199 , 200 , 200))
                    MainSquare[i][j].sqColor = Color::White;
            }
        }
        if(!ready)
        {
            //  -------ONSCREEN TEXT------------
            
            if(PointA.poz == Vector2i(-1 , -1))
            {
                TXTpoint.setString("Select point A ");
                TXTpoint.setFillColor(Color::Blue);
            }
            else if(PointA.poz != Vector2i(-1 , -1) &&
                    PointB.poz == Vector2i(-1 , -1))
            {
                TXTpoint.setString("Select point B ");
                TXTpoint.setFillColor(Color::Red);
            }
            else if(PointA.poz != Vector2i(-1 , -1) &&
                    PointB.poz != Vector2i(-1 , -1))
            {
                TXTpoint.setString("Press E to start path search");
                TXTpoint.setFillColor(Color::White);
            }
        }
        else
        {
            TXTpoint.setScale(0.7, 0.7);
            TXTpoint.setString("executing");
        }
        if(Event::KeyPressed)
        {
            if(event.key.code == sf::Keyboard::Key::E
               && PointA.poz != Vector2i(-1 , -1)
               && PointB.poz != Vector2i(-1 , -1))
            {
                ready = true;
            }
        }
        // ---------DRAW Everything (besides the squares 😄)---------
        window.draw(TXTpoint);
        window.draw(TXTCredits);
        window.display();
    }
}
