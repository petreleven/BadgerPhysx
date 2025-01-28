#include "Core.h"
#include "ForceGenerator.h"
#include "World.h"
#include <raylib.h>
#include <rlgl.h>

void DrawOrientedCube(const badger::RigidBody &body, float width, float height,
                      float depth, Color color);

int main() {
  badger::RigidBody b;
  badger::real mass = 10;
  badger::real height = 100;
  badger::real width = 100;
  badger::real depth = 100;
  b.setPosition(badger::Vector3(200, 200, 0)); // Increased y value to 280
  b.setMass(mass);
  b.setsOrientation(badger::Quaternion(1, 0, 0, 0));
  b.SetDimensions(height, width, depth);
  badger::GravityForceGenerator g(badger::Vector3(0,-4000, 0));
  badger::World physixWorld;
  physixWorld.registerBody(&b);
  physixWorld.registerBodyForce(&b, &g);

  const int screenWidth = 800;
  const int screenHeight = 450;
  InitWindow(screenWidth, screenHeight, "badgerPhysX 3D");
  // Camera setup
  Camera3D camera = {0};
  camera.position = (Vector3){200.0f, 200.0f, 400.0f};
  camera.target = (Vector3){200.0f, 200.0f, 0.0f};
  camera.up = (Vector3){0.0f, 1.0f, 0.0f};
  camera.fovy = 45.0f;
  camera.projection = CAMERA_PERSPECTIVE;

  // Camera orbital controls
  float cameraAngle = 0.0f;
  float cameraPitch = 0.0f;
  float cameraDistance = 500.0f;
  Vector3 cameraTarget = {200.0f, 220.0f, 0.0f}; // Center of the scene

  SetTargetFPS(60);

  while (!WindowShouldClose()) {
    {
      // Update camera position based on orbital controls
      if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        cameraAngle += GetMouseDelta().x * 0.5f;
        cameraPitch += GetMouseDelta().y * 0.5f;

        // Limit pitch angle to avoid camera flipping
        if (cameraPitch > 89.0f)
          cameraPitch = 89.0f;
        if (cameraPitch < -89.0f)
          cameraPitch = -89.0f;
      }

      // Zoom with mouse wheel
      float wheelMove = GetMouseWheelMove();
      if (wheelMove != 0) {
        cameraDistance -= wheelMove * 20.0f;
        if (cameraDistance < 100.0f)
          cameraDistance = 100.0f;
        if (cameraDistance > 1000.0f)
          cameraDistance = 1000.0f;
      }

      // Calculate camera position based on spherical coordinates
      float pitchRad = cameraPitch * DEG2RAD;
      float angleRad = cameraAngle * DEG2RAD;

      camera.position.x =
          cameraTarget.x + cameraDistance * cosf(pitchRad) * sinf(angleRad);
      camera.position.y = cameraTarget.y + cameraDistance * sinf(pitchRad);
      camera.position.z =
          cameraTarget.z + cameraDistance * cosf(pitchRad) * cosf(angleRad);
      camera.target = cameraTarget;
    }
    // physics
    {
      physixWorld.startFrame();
      physixWorld.update(.016);
    }
    // rendering
    {
      BeginDrawing();
      ClearBackground(Color{100, 100, 100});
      BeginMode3D(camera);
      DrawOrientedCube(b, width, height, depth, GREEN);
      DrawGrid(100, 50.0f);
      EndMode3D();

      // Draw controls info
      DrawFPS(10, 10);
      DrawText("Left Mouse Button: Rotate Camera", 10, 30, 20, DARKGRAY);
      DrawText("Mouse Wheel: Zoom", 10, 50, 20, DARKGRAY);
      EndDrawing();
    }
  }

  CloseWindow();
  return 0;
}

void DrawOrientedCube(const badger::RigidBody &body, float width, float height,
                      float depth, Color color) {
  // Convert quaternion to rotation matrix
  badger::Matrix4 rotMat = body.getTransform();
  badger::Vector3 position = body.getPosition();
  // Create raylib matrix (note: raylib uses column-major order)
  float matTransform[16] = {
      rotMat.data[0], rotMat.data[4], rotMat.data[8],  0.0f,
      rotMat.data[1], rotMat.data[5], rotMat.data[9],  0.0f,
      rotMat.data[2], rotMat.data[6], rotMat.data[10], 0.0f,
      position.x,     position.y,     position.z,      1.0f};

  // Save current matrix
  rlPushMatrix();

  // Apply our transformation
  rlMultMatrixf(matTransform);

  // Draw cube centered at origin (since the transform handles position)
  DrawCube((Vector3){0, 0, 0}, width + 1, height + 1, depth + 1, color);
  // If you want wireframe, add:
  DrawCubeWires((Vector3){0, 0, 0}, width + 1, height + 1, depth + 1,
                ColorAlpha(BLACK, 0.3f));
  // DrawSphere(Vector3{0, 0, 0}, height, color);
  // DrawSphereWires(Vector3{0, 0, 0}, height + 4, 4, 4, ColorAlpha(BLACK,
  // 0.3f));

  // Restore previous matrix
  rlPopMatrix();
}
