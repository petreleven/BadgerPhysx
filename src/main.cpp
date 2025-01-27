#include "Core.h"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <raylib.h>
/*
does stuff
*/
void eg(){};

int main() {
    badger::Vector3 p(400, 0, 0);
    badger::real angle = DEG2RAD * 45 ;
    badger::Vector3 v(0, 0, 1);
    badger::Quaternion q(cos(angle/2), v.x*sin(angle/2), v.y*sin(angle/2), v.z*sin(angle/2));
    q.normalize();
    badger::Quaternion newPasQ = q * badger::Quaternion(0, p.x, p.y, p.z) * badger::Quaternion(q.r, -q.i, -q.j, -q.k);
    badger::Vector3 newPV(newPasQ.i, newPasQ.j, newPasQ.k);
    std::cout<<"x:"<<newPV.x<<" y:"<<newPV.y<<" z:"<<newPV.z<<"\n";
  // Initialization
  //--------------------------------------------------------------------------------------
  const int screenWidth = 800;
  const int screenHeight = 450;

  InitWindow(screenWidth, screenHeight, "test");

  SetTargetFPS(60); // Set our game to run at 60 frames-per-second
  //--------------------------------------------------------------------------------------

  // Main game loop
  while (!WindowShouldClose()) // Detect window close button or ESC key
  {
    // Update
    //----------------------------------------------------------------------------------
    // TODO: Update your variables here
    //----------------------------------------------------------------------------------

    // Draw
    //----------------------------------------------------------------------------------
    BeginDrawing();
    DrawLine(0, 0, p.x, p.y, GREEN);
    DrawLine(0, 0, newPV.x, newPV.y, RED);
    DrawCircle(p.x, p.y, 10, GREEN);
    DrawCircle(newPV.x, newPV.y, 10, RED);

    ClearBackground(Color{26, 26, 29});
    EndDrawing();
    //----------------------------------------------------------------------------------
  }

  // De-Initialization
  //--------------------------------------------------------------------------------------
  CloseWindow(); // Close window and OpenGL context
  //--------------------------------------------------------------------------------------
}
