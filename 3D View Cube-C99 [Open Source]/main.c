// 3D View Cube.
// Example originally created with raylib 5.5
// Example contributed by Niklas Holmberg (@Nightfall800)
// Date 2025-09-14. 

/*
MIT License

Copyright (c) 2025 Niklas Holmberg.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>

// === Utils ===

static float clampf(float v, float lo, float hi) { return (v < lo ? lo : (v > hi ? hi : v)); }

static float Distance3D(Vector3 a, Vector3 b) {
    return Vector3Length(Vector3Subtract(a, b));
}

// === Angle helpers (degrees) ===
static float WrapAngle360(float a) {
    float r = fmodf(a, 360.0f);
    if (r < 0.0f) r += 360.0f;
    return r;
}
static float WrapAngle180(float a) {
    float r = WrapAngle360(a);
    if (r > 180.0f) r -= 360.0f;
    return r; // (-180, +180]
}
static float MoveTowardAngle(float current, float target, float maxStepDeg) {
    float cur = WrapAngle360(current);
    float tgt = WrapAngle360(target);
    float delta = WrapAngle180(tgt - cur);
    if (fabsf(delta) <= maxStepDeg) return tgt;
    return WrapAngle360(cur + ((delta > 0.0f) ? maxStepDeg : -maxStepDeg));
}

// === View cube rotation helpers ===

static void PushYPR(float yawDeg, float pitchDeg, float rollDeg) {
    rlPushMatrix();
    rlRotatef(yawDeg, 0.0f, 1.0f, 0.0f); // Yaw
    rlRotatef(pitchDeg, 1.0f, 0.0f, 0.0f); // Pitch
    rlRotatef(rollDeg, 0.0f, 0.0f, 1.0f); // Roll
}
static Matrix MatrixYPR(float yawDeg, float pitchDeg, float rollDeg) {
    Matrix ry = MatrixRotateY(DEG2RAD * yawDeg);
    Matrix rx = MatrixRotateX(DEG2RAD * pitchDeg);
    Matrix rz = MatrixRotateZ(DEG2RAD * rollDeg);
    return MatrixMultiply(ry, MatrixMultiply(rx, rz)); // yaw -> pitch -> roll
}

// === Transform a world-space ray to cube-local space via inverse(M) ===
static Ray TransformRayInverse(Ray ray, Matrix m) {
    Matrix inv = MatrixInvert(m);
    Vector3 posLocal = Vector3Transform(ray.position, inv);
    Vector3 endWorld = Vector3Add(ray.position, Vector3Scale(ray.direction, 1000.0f));
    Vector3 endLocal = Vector3Transform(endWorld, inv);
    Vector3 dirLocal = Vector3Normalize(Vector3Subtract(endLocal, posLocal));
    Ray r;
    r.position = posLocal;
    r.direction = dirLocal;
    return r;
}

// === Ray vs plane Y = const (true if hit & sets 'out') ===
static bool RayHitPlaneY(Ray r, float yConst, Vector3* out) {
    float dy = r.direction.y;
    if (fabsf(dy) < 1e-6f) return false; // nearly parallel
    float t = (yConst - r.position.y) / dy;
    if (t < 0.0f) return false; // behind camera
    Vector3 p = Vector3Add(r.position, Vector3Scale(r.direction, t));
    if (out) *out = p;
    return true;
}

// === Scene camera helpers ===

static Vector3 ChooseUpForDir(Vector3 dir) {
    Vector3 up = (Vector3){ 0.0f, 1.0f, 0.0f };
    if (fabsf(dir.y) > 0.95f) up = (Vector3){ 0.0f, 0.0f, 1.0f };
    return up;
}
static void LookFromDirection(Camera3D* cam, Vector3 dir) {
    dir = Vector3Normalize(dir);
    float d = Distance3D(cam->position, cam->target);
    cam->position = Vector3Add(cam->target, Vector3Scale(dir, d));
    cam->up = ChooseUpForDir(Vector3Negate(dir));
}

// === Picking (faces only) ===

typedef enum PickType { PT_None, PT_Face } PickType;

typedef struct PickResult {
    PickType type;
    int index;
    float distance;
} PickResult;

static PickResult PickResultMake(PickType type, int index, float distance) {
    PickResult r; r.type = type; r.index = index; r.distance = distance; return r;
}

// === 0..5 => +X,-X,+Y,-Y,+Z,-Z ===
static int DominantFaceFromDir(Vector3 dir) {
    float ax = fabsf(dir.x), ay = fabsf(dir.y), az = fabsf(dir.z);
    if (ax >= ay && ax >= az) return (dir.x >= 0.0f) ? 0 : 1;
    if (ay >= ax && ay >= az) return (dir.y >= 0.0f) ? 2 : 3;
    return (dir.z >= ax && dir.z >= ay) ? ((dir.z >= 0.0f) ? 4 : 5) : ((dir.z >= 0.0f) ? 4 : 5);
}

static Ray GetMouseRayInRect(Vector2 mouse, Rectangle rect, Camera3D cam) {
    Vector2 local = (Vector2){
        (mouse.x - rect.x) / rect.width,
        (mouse.y - rect.y) / rect.height
    };
    local.x = clampf(local.x, 0.0f, 1.0f);
    local.y = clampf(local.y, 0.0f, 1.0f);
    Vector2 fakeScreen = (Vector2){ local.x * (float)GetScreenWidth(), local.y * (float)GetScreenHeight() };
    return GetMouseRay(fakeScreen, cam);
}

// === Project world->screen and clamp to window (prevents SetMousePosition outside) ===
static Vector2 ProjectToScreenClamped(Vector3 p, Camera3D cam) {
    Vector2 s = GetWorldToScreen(p, cam);
    int sw = GetScreenWidth();
    int sh = GetScreenHeight();

    // Guard against NaN/Inf
    if (!isfinite((double)s.x) || !isfinite((double)s.y)) {
        // Fallback: center of window
        return (Vector2) { (float)sw * 0.5f, (float)sh * 0.5f };
    }
    // Clamp within [0 .. width-1], [0 .. height-1]
    s.x = clampf(s.x, 0.0f, (float)(sw - 1));
    s.y = clampf(s.y, 0.0f, (float)(sh - 1));
    return s;
}

// === Warp mouse to world-point projection in WINDOW coords ===
static void WarpMouseToWorld(Vector3 p, Camera3D cam) {
    Vector2 sWin = ProjectToScreenClamped(p, cam);   // window pixels
    SetMousePosition((int)lroundf(sWin.x), (int)lroundf(sWin.y));
}

// === Faces only: thick slabs -> easy to hit ===
static PickResult PickOnViewCube(Ray rayLocal) {
    const float half = 0.5f;
    const float feps = 0.12f; // big hit area

    BoundingBox faces[6] = {
        { (Vector3) { +half - feps, -half, -half }, (Vector3) { +half + feps, +half, +half } }, // +X
        { (Vector3) { -half - feps, -half, -half }, (Vector3) { -half + feps, +half, +half } }, // -X
        { (Vector3) { -half, +half - feps, -half }, (Vector3) { +half, +half + feps, +half } }, // +Y
        { (Vector3) { -half, -half - feps, -half }, (Vector3) { +half, -half + feps, +half } }, // -Y
        { (Vector3) { -half, -half, +half - feps }, (Vector3) { +half, +half, +half + feps } }, // +Z
        { (Vector3) { -half, -half, -half - feps }, (Vector3) { +half, +half, -half + feps } }, // -Z
    };

    float bestDist = 1e9f;
    int   bestFace = -1;
    for (int i = 0; i < 6; ++i) {
        RayCollision c = GetRayCollisionBox(rayLocal, faces[i]);
        if (c.hit && c.distance < bestDist) {
            bestDist = c.distance;
            bestFace = i;
        }
    }
    if (bestFace >= 0) return PickResultMake(PT_Face, bestFace, bestDist);
    return PickResultMake(PT_None, -1, 1e9f);
}

static Vector3 FaceDir(int f) {
    switch (f) {
    case 0: return (Vector3) { +1, 0, 0 };
    case 1: return (Vector3) { -1, 0, 0 };
    case 2: return (Vector3) { 0, +1, 0 };
    case 3: return (Vector3) { 0, -1, 0 };
    case 4: return (Vector3) { 0, 0, +1 };
    case 5: return (Vector3) { 0, 0, -1 };
    default: return (Vector3) { 0, 0, +1 };
    }
}

// === Labels ===

// === Large label with black text + white outline ===
static Texture2D MakeFaceLabelTexture(const char* txt, int w, int h, int fontSize) {
    Image img = GenImageColor(w, h, BLANK);
    int tw = MeasureText(txt, fontSize);
    int x = (w - tw) / 2;
    int y = (h - fontSize) / 2;

    // White outline
    for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
            if (dx == 0 && dy == 0) continue;
            ImageDrawText(&img, txt, x + dx, y + dy, fontSize, WHITE);
        }
    }
    // Black text
    ImageDrawText(&img, txt, x, y, fontSize, BLACK);

    Texture2D tex = LoadTextureFromImage(img);
    SetTextureFilter(tex, TEXTURE_FILTER_POINT); // crisp when small
    UnloadImage(img);
    return tex;
}

// Draw label "attached" to face but always facing the screen.
// nLocal = face normal in cube local space (e.g., {0,+1,0} for TOP).
static void DrawLabelFaceOriented(Texture2D tex, Matrix R, Camera3D cam, Vector3 nLocal, float offset) {
    // 1) world-normal after cube model rotation
    Vector3 nW = Vector3Normalize(Vector3Transform(nLocal, R));

    // 2) Choose face-up (vW) close to camera up (projected in plane)
    Vector3 camUp = cam.up;
    Vector3 vW = Vector3Subtract(camUp, Vector3Scale(nW, Vector3DotProduct(camUp, nW))); // projection
    if (Vector3Length(vW) < 1e-4f) {
        // Fallback: use camera direction
        Vector3 camDir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
        vW = Vector3CrossProduct(nW, camDir);
        if (Vector3Length(vW) < 1e-4f) vW = (Vector3){ 0,1,0 };
    }
    vW = Vector3Normalize(vW);
    Vector3 uW = Vector3Normalize(Vector3CrossProduct(vW, nW)); // face-right

    // 3) Build corners in world-space (center = (0.5 + offset) * nW)
    const float h = 0.5f;
    Vector3 center = Vector3Scale(nW, h + offset);
    Vector3 pBL = Vector3Add(center, Vector3Add(Vector3Scale(uW, -h), Vector3Scale(vW, -h)));
    Vector3 pBR = Vector3Add(center, Vector3Add(Vector3Scale(uW, h), Vector3Scale(vW, -h)));
    Vector3 pTR = Vector3Add(center, Vector3Add(Vector3Scale(uW, h), Vector3Scale(vW, h)));
    Vector3 pTL = Vector3Add(center, Vector3Add(Vector3Scale(uW, -h), Vector3Scale(vW, h)));

    // 4) Draw in world coords (model matrix = identity), on top of everything
    rlPushMatrix();
    rlLoadIdentity();
    rlSetTexture(tex.id);
    rlBegin(RL_QUADS);
    rlTexCoord2f(0, 1); rlVertex3f(pBL.x, pBL.y, pBL.z);
    rlTexCoord2f(1, 1); rlVertex3f(pBR.x, pBR.y, pBR.z);
    rlTexCoord2f(1, 0); rlVertex3f(pTR.x, pTR.y, pTR.z);
    rlTexCoord2f(0, 0); rlVertex3f(pTL.x, pTL.y, pTL.z);
    rlEnd();
    rlSetTexture(0);
    rlPopMatrix();
}

// === CUBES data ===
typedef struct Cube {
    int     id;
    Vector3 pos;
    bool    alive;
} Cube;

typedef struct CubeArray {
    Cube* data;
    int size;
    int capacity;
} CubeArray;

static void cubes_init(CubeArray* a) {
    a->size = 0;
    a->capacity = 8;
    a->data = (Cube*)malloc(sizeof(Cube) * a->capacity);
}
static void cubes_free(CubeArray* a) {
    free(a->data);
    a->data = NULL; a->size = a->capacity = 0;
}
static void cubes_push(CubeArray* a, Cube c) {
    if (a->size >= a->capacity) {
        a->capacity *= 2;
        a->data = (Cube*)realloc(a->data, sizeof(Cube) * a->capacity);
    }
    a->data[a->size++] = c;
}

// === Main ===

static float nearest90(float aDeg) {
    float a = WrapAngle360(aDeg);
    float k = roundf(a / 90.0f);
    return WrapAngle360(k * 90.0f);
}

int main(void) {
    const int screenWidth = 1280;
    const int screenHeight = 720;
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(screenWidth, screenHeight, "3D View Cube");

    // Scene camera
    Camera3D camera = { 0 };
    camera.position = (Vector3){ 10.0f, 10.0f, 10.0f };
    camera.target = (Vector3){ 0.0f,  0.0f,  0.0f };
    camera.up = (Vector3){ 0.0f,  1.0f,  0.0f };
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // Mini camera (fixed pose showing top/right/front clearly)
    Camera3D miniCam = { 0 };
    miniCam.position = (Vector3){ 2.6f, 3.4f, 1.9f };
    miniCam.target = (Vector3){ 0.0f, 0.0f, 0.0f };
    miniCam.up = (Vector3){ 0.0f, 1.0f, 0.0f };
    miniCam.fovy = 35.0f;
    miniCam.projection = CAMERA_PERSPECTIVE;

    // RenderTexture for view cube (transparent)
    const int VC_SIZE = 288;    // View Cube SIZE. Large so labels are crisp
    RenderTexture2D vcRT = LoadRenderTexture(VC_SIZE, VC_SIZE);
    SetTextureFilter(vcRT.texture, TEXTURE_FILTER_BILINEAR);

    // Colors
    const Color BASE_DARK = (Color){ 20, 40, 100, 255 };  // medium-dark blue
    const Color FACE_LIT = (Color){ 70, 120, 220, 255 }; // brighter blue
    const Color WIRES = DARKBLUE;

    // Labels (high res, big fonts)
    const int LABEL_W = 1024, LABEL_H = 1024;
    Texture2D texTop = MakeFaceLabelTexture("TOP", LABEL_W, LABEL_H, 320);
    Texture2D texBottom = MakeFaceLabelTexture("BOTTOM", LABEL_W, LABEL_H, 280);
    Texture2D texLeft = MakeFaceLabelTexture("LEFT", LABEL_W, LABEL_H, 300);
    Texture2D texRight = MakeFaceLabelTexture("RIGHT", LABEL_W, LABEL_H, 300);
    Texture2D texFront = MakeFaceLabelTexture("FRONT", LABEL_W, LABEL_H, 300);
    Texture2D texBack = MakeFaceLabelTexture("BACK", LABEL_W, LABEL_H, 300);

    bool rotatingWithRMB = false;    // free camera hold

    // CUBES
    CubeArray cubes; cubes_init(&cubes);
    int selectedIdx = -1;   // index in 'cubes', -1 = none
    int nextId = 1;         // ID counter
    // Start with one cube at origin (optional)
    cubes_push(&cubes, (Cube) { nextId++, (Vector3) { 0, 0, 0 }, true });
    selectedIdx = -1; // none selected at start

    EnableCursor();
    SetTargetFPS(60);

    // === MOVE state ===
    bool moveMode = false;           // if we are moving an object
    float  moveRefY = 0.0f;          // locked Y in XZ-plane mode
    Vector2 movePrevMouse = (Vector2){ 0,0 };    // for Y-change with SHIFT
    Vector3 moveGrabDelta = (Vector3){ 0,0,0 };  // offset between object and ray hit
    bool moveJustWarped = false;     // skip first update step
    Vector2 moveWarpPos = (Vector2){ 0,0 };      // from GetWorldToScreen(cubePosition)
    Vector3 moveLockPos = (Vector3){ 0,0,0 };    // exact pos at warp (lock for 1 frame)
    bool moveAwaitFirstDelta = false;  // wait until user moves mouse
    bool moveShiftWasDown = false;     // remember if SHIFT was down last frame

    // View cube rotation state (user scrolls), infinite + snapping
    float vcYaw = 0.0f, vcPitch = 0.0f, vcRoll = 0.0f;
    float vcYawTarget = 0.0f, vcPitchTarget = 0.0f;
    const float dtClamp = 1.0f / 30.0f;
    float vcHoldTimer = 0.0f;     // pause before snapping
    const float snapSpeed = 180.0f; // deg/s toward nearest 90°
    const float snapEps = 0.5f;   // epsilon

    while (!WindowShouldClose()) {
        float dt = fminf(GetFrameTime(), dtClamp);

        // --- View cube rect (top-right) ---
        const int margin = 16;
        int sw = GetScreenWidth();
        Rectangle vcRect = (Rectangle){ (float)(sw - VC_SIZE - margin), (float)margin, (float)VC_SIZE, (float)VC_SIZE };

        // --- Mouse ---
        Vector2 mouse = GetMousePosition();
        bool insideVC = CheckCollisionPointRec(mouse, vcRect);
        float wheelDelta = GetMouseWheelMove();

        // --- Scene camera input (blocked over view cube and while moving) ---
        if (!moveMode && IsMouseButtonPressed(MOUSE_RIGHT_BUTTON)) { rotatingWithRMB = true;  DisableCursor(); }
        if (IsMouseButtonReleased(MOUSE_RIGHT_BUTTON)) { rotatingWithRMB = false; EnableCursor(); }

        if (rotatingWithRMB && !moveMode) {
            UpdateCamera(&camera, CAMERA_FREE);
        }
        else if (!moveMode) {
            if (!insideVC && wheelDelta != 0.0f) {
                Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
                camera.position = Vector3Add(camera.position, Vector3Scale(forward, wheelDelta * 1.0f));
            }
            if (!insideVC && IsMouseButtonDown(MOUSE_MIDDLE_BUTTON)) {
                Vector2 d = GetMouseDelta();
                Vector3 dir = Vector3Subtract(camera.target, camera.position);
                Vector3 right = Vector3Normalize(Vector3CrossProduct(dir, camera.up));
                Vector3 up = camera.up;
                const float pan = 0.01f;
                camera.position = Vector3Add(camera.position, Vector3Scale(right, -d.x * pan));
                camera.target = Vector3Add(camera.target, Vector3Scale(right, -d.x * pan));
                camera.position = Vector3Add(camera.position, Vector3Scale(up, d.y * pan));
                camera.target = Vector3Add(camera.target, Vector3Scale(up, d.y * pan));
            }
        }
        if (IsKeyPressed(KEY_Z)) camera.target = (Vector3){ 0.0f, 0.0f, 0.0f };

        // A = add new cube at origin
        if (IsKeyPressed(KEY_A)) {
            cubes_push(&cubes, (Cube) { nextId++, (Vector3) { 0, 0, 0 }, true });
            selectedIdx = cubes.size - 1; // select newly created
        }

        // Active face from scene view
        Vector3 sceneDir = Vector3Normalize(Vector3Subtract(camera.position, camera.target)); // target->camera
        int activeFace = DominantFaceFromDir(sceneDir);

        // --- View cube rotate input (ONLY over cube) ---
        if (insideVC && wheelDelta != 0.0f) {
            if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
                vcYaw += wheelDelta * 20.0f;          // Shift + Wheel = Yaw
            }
            else {
                vcPitch -= wheelDelta * 20.0f;        // Wheel = Pitch, TOP towards you
            }
            vcHoldTimer = 0.5f;                     // pause before snap
            vcYawTarget = vcYaw;
            vcPitchTarget = vcPitch;
        }

        // --- Click on faces only ---
        if (insideVC && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Ray rayScreen = GetMouseRayInRect(mouse, vcRect, miniCam);
            Matrix M = MatrixYPR(vcYaw, vcPitch, vcRoll);
            Ray rayLocal = TransformRayInverse(rayScreen, M);

            PickResult pr = PickOnViewCube(rayLocal);
            if (pr.type == PT_Face) {
                LookFromDirection(&camera, FaceDir(pr.index));
                vcHoldTimer = 0.0f; // snap right after choosing
            }
        }

        // --- Click in SCENE (NOT over view cube) ---
        if (!insideVC && IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
            Ray ray = GetMouseRay(mouse, camera);

            int bestIdx = -1;
            int bestId = 0x7fffffff;   // large
            const Vector3 halfExt = (Vector3){ 0.5f, 0.5f, 0.5f };

            for (int i = 0; i < cubes.size; ++i) {
                if (!cubes.data[i].alive) continue;
                BoundingBox box = (BoundingBox){
                    Vector3Subtract(cubes.data[i].pos, halfExt),
                    Vector3Add(cubes.data[i].pos, halfExt)
                };
                RayCollision hit = GetRayCollisionBox(ray, box);
                if (hit.hit) {
                    if (cubes.data[i].id < bestId) { // lowest ID wins
                        bestId = cubes.data[i].id;
                        bestIdx = i;
                    }
                }
            }
            selectedIdx = bestIdx; // -1 = none hit
        }

        // --- Start MOVE-mode (M) when an object is selected ---
        if (selectedIdx != -1 && cubes.data[selectedIdx].alive && IsKeyPressed(KEY_M)) {
            moveMode = true;

            Vector3* p = &cubes.data[selectedIdx].pos;
            moveRefY = p->y;

            // Cursor must be visible before warp
            rotatingWithRMB = false;
            EnableCursor();

            // Warp mouse EXACTLY to cube's screen point
            WarpMouseToWorld(*p, camera);

            // Read back pointer position now (window-XY)
            Vector2 mNow = GetMousePosition();
            moveWarpPos = mNow;     // where we landed
            movePrevMouse = mNow;   // reference for first real move

            // Lock initial position
            moveLockPos = *p;
            moveGrabDelta = (Vector3){ 0,0,0 };

            // Skip first update & wait for first real delta
            moveJustWarped = true;
            moveAwaitFirstDelta = true;

            // Start with current SHIFT state
            moveShiftWasDown = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT));
        }


        // --- Snap toward nearest 90° after hold ---
        if (vcHoldTimer > 0.0f) {
            vcHoldTimer -= dt;
            if (vcHoldTimer < 0.0f) vcHoldTimer = 0.0f;
        }
        else {
            vcYawTarget = nearest90(vcYaw);
            vcPitchTarget = nearest90(vcPitch);

            float maxStep = snapSpeed * dt;
            vcYaw = MoveTowardAngle(vcYaw, vcYawTarget, maxStep);
            vcPitch = MoveTowardAngle(vcPitch, vcPitchTarget, maxStep);

            if (fabsf(WrapAngle180(vcYaw - vcYawTarget)) < snapEps) vcYaw = vcYawTarget;
            if (fabsf(WrapAngle180(vcPitch - vcPitchTarget)) < snapEps) vcPitch = vcPitchTarget;
        }

        // --- Delete key removes selected object ---
        if (selectedIdx != -1 && cubes.data[selectedIdx].alive &&
            (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE))) {
            cubes.data[selectedIdx].alive = false;
            selectedIdx = -1;
        }

        // --- MOVE-mode update ---
        if (moveMode) {
            // 1) Finish with left click -> exit move and deselect
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                moveMode = false;
                selectedIdx = -1;
            }
            else {
                // Nothing to move? Abort safely.
                if (selectedIdx == -1 || !cubes.data[selectedIdx].alive) {
                    moveMode = false;
                }
                else {
                    Vector3* P = &cubes.data[selectedIdx].pos;

                    // Step A: first frame after warp -> lock & sync
                    if (moveJustWarped) {
                        moveJustWarped = false;
                        *P = moveLockPos;            // keep exact start position
                        movePrevMouse = moveWarpPos; // sync reference
                        // do nothing else this frame
                    }
                    // Step B: wait for actual mouse movement
                    else if (moveAwaitFirstDelta) {
                        Vector2 d0 = GetMouseDelta();
                        if (fabsf(d0.x) < 0.0001f && fabsf(d0.y) < 0.0001f) {
                            // wait until user moves mouse
                        }
                        else {
                            moveAwaitFirstDelta = false;
                            movePrevMouse = GetMousePosition();
                        }
                    }
                    // Step C: normal move update
                    else {
                        Vector2 mNow = GetMousePosition();

                        // --- Handle SHIFT state transitions ---
                        bool shiftDown = (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT));
                        if (shiftDown != moveShiftWasDown) {
                            if (shiftDown) {
                                // SHIFT JUST PRESSED: zero Y-delta to avoid jump
                                movePrevMouse = mNow;  // dy = 0
                            }
                            else {
                                // SHIFT JUST RELEASED:
                                // 1) Warp pointer to cube's CURRENT screen point
                                WarpMouseToWorld(*P, camera);
                                // 2) Read back exact pointer pos (window-XY)
                                Vector2 mSync = GetMousePosition();
                                // 3) Compute grabDelta AFTER warp so hit + delta == P
                                Ray r1 = GetMouseRay(mSync, camera);
                                Vector3 h1 = (Vector3){ 0 };
                                if (RayHitPlaneY(r1, moveRefY, &h1)) {
                                    moveGrabDelta.x = P->x - h1.x;
                                    moveGrabDelta.z = P->z - h1.z;
                                }
                                // 4) Sync references and UPDATE mNow so this frame uses it
                                movePrevMouse = mSync;
                                moveWarpPos = mSync;
                                mNow = mSync;   // important
                            }
                            moveShiftWasDown = shiftDown;
                        }
                        // -----------------------------------------

                        if (shiftDown) {
                            // --- Y-move (ACCUMULATED) ---
                            float dy = mNow.y - movePrevMouse.y;
                            float dist = Vector3Length(Vector3Subtract(camera.position, *P));
                            float yScale = 0.01f * dist;
                            P->y += (-dy) * yScale;   // accumulate
                            moveRefY = P->y;          // XZ-plane follows new height
                        }
                        else {
                            // --- XZ-move: plane Y = moveRefY ---
                            Ray r = GetMouseRay(mNow, camera);
                            Vector3 hit = (Vector3){ 0 };
                            if (RayHitPlaneY(r, moveRefY, &hit)) {
                                P->x = hit.x + moveGrabDelta.x;
                                P->z = hit.z + moveGrabDelta.z;
                            }
                        }

                        movePrevMouse = mNow;
                    }
                }
            }
        }

        // ---- Draw ----
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Scene
        BeginMode3D(camera);

        // Draw only if object is alive
        for (int i = 0; i < cubes.size; ++i) {
            if (!cubes.data[i].alive) continue;
            Vector3 p = cubes.data[i].pos;

            bool isSelected = (i == selectedIdx);
            DrawCube(p, 1.0f, 1.0f, 1.0f, RED);

            float lw = isSelected ? 1.2f : 1.0f;
            Color wireCol = isSelected ? GREEN : MAROON;

            rlSetLineWidth(lw);
            DrawCubeWires(p, 1.0f, 1.0f, 1.0f, wireCol);
            rlSetLineWidth(1.0f);

            if (isSelected) {
                DrawCubeWires(p, 1.006f, 1.006f, 1.006f, wireCol);
            }
        }

        DrawGrid(50, 1.0f);
        EndMode3D();

        // View cube (render to texture)
        BeginTextureMode(vcRT);
        ClearBackground(BLANK);
        BeginMode3D(miniCam);
        // Rotated model of the cube
        PushYPR(vcYaw, vcPitch, vcRoll);

        // Base cube
        DrawCube((Vector3) { 0, 0, 0 }, 1, 1, 1, BASE_DARK);
        DrawCubeWires((Vector3) { 0, 0, 0 }, 1, 1, 1, WIRES);

        // Highlight active face (thin inset plate)
        const float half = 0.5f, t = 0.02f, inset = 0.02f;
        switch (activeFace) {
        case 0: DrawCube((Vector3) { +half - t / 2, 0.0f, 0.0f }, t, 1.0f - inset * 2, 1.0f - inset * 2, FACE_LIT); break; // +X
        case 1: DrawCube((Vector3) { -half + t / 2, 0.0f, 0.0f }, t, 1.0f - inset * 2, 1.0f - inset * 2, FACE_LIT); break; // -X
        case 2: DrawCube((Vector3) { 0.0f, +half - t / 2, 0.0f }, 1.0f - inset * 2, t, 1.0f - inset * 2, FACE_LIT); break; // +Y
        case 3: DrawCube((Vector3) { 0.0f, -half + t / 2, 0.0f }, 1.0f - inset * 2, t, 1.0f - inset * 2, FACE_LIT); break; // -Y
        case 4: DrawCube((Vector3) { 0.0f, 0.0f, +half - t / 2 }, 1.0f - inset * 2, 1.0f - inset * 2, t, FACE_LIT); break; // +Z
        case 5: DrawCube((Vector3) { 0.0f, 0.0f, -half + t / 2 }, 1.0f - inset * 2, 1.0f - inset * 2, t, FACE_LIT); break; // -Z
        }

        rlPopMatrix(); // from PushYPR

        // --- Labels: on top, always facing correctly ---
        rlDisableDepthTest();
        rlDisableBackfaceCulling();

        Matrix R = MatrixYPR(vcYaw, vcPitch, vcRoll);
        const float labelOffset = 0.02f;

        DrawLabelFaceOriented(texTop, R, miniCam, (Vector3) { 0, +1, 0 }, labelOffset);
        DrawLabelFaceOriented(texBottom, R, miniCam, (Vector3) { 0, -1, 0 }, labelOffset);
        DrawLabelFaceOriented(texRight, R, miniCam, (Vector3) { +1, 0, 0 }, labelOffset);
        DrawLabelFaceOriented(texLeft, R, miniCam, (Vector3) { -1, 0, 0 }, labelOffset);
        DrawLabelFaceOriented(texFront, R, miniCam, (Vector3) { 0, 0, +1 }, labelOffset);
        DrawLabelFaceOriented(texBack, R, miniCam, (Vector3) { 0, 0, -1 }, labelOffset);

        rlEnableBackfaceCulling();
        rlEnableDepthTest();

        EndMode3D();
        EndTextureMode();

        // Blit RT (render textures are vertically flipped)
        Rectangle src = (Rectangle){ 0, 0, (float)vcRT.texture.width, -(float)vcRT.texture.height };
        DrawTexturePro(vcRT.texture, src, vcRect, (Vector2) { 0, 0 }, 0.0f, WHITE);

        // Help text
        DrawRectangle(10, 10, 500, 170, Fade(SKYBLUE, 0.5f));
        DrawRectangleLines(10, 10, 500, 170, BLUE);
        DrawText("Camera and controls:", 20, 20, 10, BLACK);
        DrawText("- Hold Right Mouse: rotate scene (cursor captured)", 40, 40, 10, DARKGRAY);
        DrawText("- Mouse Wheel: zoom scene (except over view cube)", 40, 60, 10, DARKGRAY);
        DrawText("- Over view cube: Wheel=Pitch (rolls), Shift+Wheel=Yaw (spins)", 40, 80, 10, DARKGRAY);
        DrawText("- Click view-cube face to set view", 40, 100, 10, DARKGRAY);
        DrawText("- Select object: press M to Move; mouse=XZ, hold SHIFT for Y; LMB to confirm", 40, 120, 10, DARKGRAY);
        DrawText("- Select object: press DELETE to remove object", 40, 140, 10, DARKGRAY);
        DrawText("- Press A to Add new cube", 40, 160, 10, DARKGRAY);

        EndDrawing();
    }

    // Cleanup
    UnloadTexture(texTop);
    UnloadTexture(texBottom);
    UnloadTexture(texLeft);
    UnloadTexture(texRight);
    UnloadTexture(texFront);
    UnloadTexture(texBack);
    UnloadRenderTexture(vcRT);

    CloseWindow();

    cubes_free(&cubes);
    return 0;
}



