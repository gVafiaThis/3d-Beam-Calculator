#pragma once

#include <raylib.h>
#include "raymath.h"
#include "VBeams.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"     // Required for UI controls




namespace {
    Vector3 GetCameraForward(Camera* camera)
    {
        return Vector3Normalize(Vector3Subtract(camera->target, camera->position));
    }

    // Returns the cameras up vector (normalized)
    // Note: The up vector might not be perpendicular to the forward vector
    Vector3 GetCameraUp(Camera* camera)
    {
        return Vector3Normalize(camera->up);
    }

    // Returns the cameras right vector (normalized)
    Vector3 GetCameraRight(Camera* camera)
    {
        Vector3 forward = GetCameraForward(camera);
        Vector3 up = GetCameraUp(camera);

        return Vector3Normalize(Vector3CrossProduct(forward, up));
    }

    // Moves the camera in its forward direction
    void CameraMoveForward(Camera* camera, float distance, bool moveInWorldPlane)
    {
        Vector3 forward = GetCameraForward(camera);

        if (moveInWorldPlane)
        {
            // Project vector onto world plane
            forward.y = 0;
            forward = Vector3Normalize(forward);
        }

        // Scale by distance
        forward = Vector3Scale(forward, distance);

        // Move position and target
        camera->position = Vector3Add(camera->position, forward);
        camera->target = Vector3Add(camera->target, forward);
    }

    // Moves the camera in its up direction
    void CameraMoveUp(Camera* camera, float distance)
    {
        Vector3 up = GetCameraUp(camera);

        // Scale by distance
        up = Vector3Scale(up, distance);

        // Move position and target
        camera->position = Vector3Add(camera->position, up);
        camera->target = Vector3Add(camera->target, up);
    }

    // Moves the camera target in its current right direction
    void CameraMoveRight(Camera* camera, float distance, bool moveInWorldPlane)
    {
        Vector3 right = GetCameraRight(camera);

        if (moveInWorldPlane)
        {
            // Project vector onto world plane
            right.y = 0;
            right = Vector3Normalize(right);
        }

        // Scale by distance
        right = Vector3Scale(right, distance);

        // Move position and target
        camera->position = Vector3Add(camera->position, right);
        camera->target = Vector3Add(camera->target, right);
    }

    // Moves the camera position closer/farther to/from the camera target
    void CameraMoveToTarget(Camera* camera, float delta)
    {
        float distance = Vector3Distance(camera->position, camera->target);

        // Apply delta
        distance += delta;

        // Distance must be greater than 0
        if (distance <= 0) distance = 0.001f;

        // Set new distance by moving the position along the forward vector
        Vector3 forward = GetCameraForward(camera);
        camera->position = Vector3Add(camera->target, Vector3Scale(forward, -distance));
    }

    // Rotates the camera around its up vector
    // Yaw is "looking left and right"
    // If rotateAroundTarget is false, the camera rotates around its position
    // Note: angle must be provided in radians
    void CameraYaw(Camera* camera, float angle, bool rotateAroundTarget)
    {
        // Rotation axis
        Vector3 up = GetCameraUp(camera);

        // View vector
        Vector3 targetPosition = Vector3Subtract(camera->target, camera->position);

        // Rotate view vector around up axis
        targetPosition = Vector3RotateByAxisAngle(targetPosition, up, angle);

        if (rotateAroundTarget)
        {
            // Move position relative to target
            camera->position = Vector3Subtract(camera->target, targetPosition);
        }
        else // rotate around camera.position
        {
            // Move target relative to position
            camera->target = Vector3Add(camera->position, targetPosition);
        }
    }


    // Rotates the camera around its right vector, pitch is "looking up and down"
    //  - lockView prevents camera overrotation (aka "somersaults")
    //  - rotateAroundTarget defines if rotation is around target or around its position
    //  - rotateUp rotates the up direction as well (typically only usefull in CAMERA_FREE)
    // NOTE: angle must be provided in radians
    void CameraPitch(Camera* camera, float angle, bool lockView, bool rotateAroundTarget, bool rotateUp)
    {
        // Up direction
        Vector3 up = GetCameraUp(camera);

        // View vector
        Vector3 targetPosition = Vector3Subtract(camera->target, camera->position);

        if (lockView)
        {
            // In these camera modes we clamp the Pitch angle
            // to allow only viewing straight up or down.

            // Clamp view up
            float maxAngleUp = Vector3Angle(up, targetPosition);
            maxAngleUp -= 0.001f; // avoid numerical errors
            if (angle > maxAngleUp) angle = maxAngleUp;

            // Clamp view down
            float maxAngleDown = Vector3Angle(Vector3Negate(up), targetPosition);
            maxAngleDown *= -1.0f; // downwards angle is negative
            maxAngleDown += 0.001f; // avoid numerical errors
            if (angle < maxAngleDown) angle = maxAngleDown;
        }

        // Rotation axis
        Vector3 right = GetCameraRight(camera);

        // Rotate view vector around right axis
        targetPosition = Vector3RotateByAxisAngle(targetPosition, right, angle);

        if (rotateAroundTarget)
        {
            // Move position relative to target
            camera->position = Vector3Subtract(camera->target, targetPosition);
        }
        else // rotate around camera.position
        {
            // Move target relative to position
            camera->target = Vector3Add(camera->position, targetPosition);
        }

        if (rotateUp)
        {
            // Rotate up direction around right axis
            camera->up = Vector3RotateByAxisAngle(camera->up, right, angle);
        }
    }

    // Rotates the camera around its forward vector
    // Roll is "turning your head sideways to the left or right"
    // Note: angle must be provided in radians
    void CameraRoll(Camera* camera, float angle)
    {
        // Rotation axis
        Vector3 forward = GetCameraForward(camera);

        // Rotate up direction around forward axis
        camera->up = Vector3RotateByAxisAngle(camera->up, forward, angle);
    }

    // Returns the camera view matrix
    Matrix GetCameraViewMatrix(Camera* camera)
    {
        return MatrixLookAt(camera->position, camera->target, camera->up);
    }

    void CameraRotateAround(Camera* camera, Vector3 target, float yawAng, float pitchAng) {
        // Rotation axis
        Vector3 up = GetCameraUp(camera);

        // Rotation axis
        Vector3 right = GetCameraRight(camera);

        // View vector
        Vector3 targetPosition = Vector3Subtract(camera->target, camera->position);

        // Move position relative to target
        camera->position = Vector3Subtract(target, Vector3RotateByAxisAngle(targetPosition, right, -pitchAng));
        //camera->up = Vector3RotateByAxisAngle(camera->up, right, -pitchAng);


        targetPosition = Vector3Subtract(camera->target, camera->position);
        camera->position = Vector3Subtract(target, Vector3RotateByAxisAngle(targetPosition, up, -yawAng));

    }

    void UpdateCameraProAroundTarget(Camera* camera, Vector3 movement, Vector3 rotation, float zoom)
    {
        // Required values
        // movement.x - Move forward/backward
        // movement.y - Move right/left
        // movement.z - Move up/down
        // rotation.x - yaw
        // rotation.y - pitch
        // rotation.z - roll
        // zoom - Move towards target

        bool lockView = true;
        bool rotateAroundTarget = true;
        bool rotateUp = false;
        bool moveInWorldPlane = true;

        // Camera rotation
        CameraPitch(camera, -rotation.y * DEG2RAD, lockView, rotateAroundTarget, rotateUp);
        CameraYaw(camera, -rotation.x * DEG2RAD, rotateAroundTarget);
        CameraRoll(camera, rotation.z * DEG2RAD);

        // Camera movement
        CameraMoveForward(camera, movement.x, moveInWorldPlane);
        CameraMoveRight(camera, movement.y, moveInWorldPlane);
        CameraMoveUp(camera, movement.z);

        // Zoom target distance
        CameraMoveToTarget(camera, zoom);
    }
}

namespace Rendering{

    void CameraControls(Vector3& rotCenter, Camera3D& camera, const BoundingBox& objectsBoundBox);
    void SetupScene(Camera3D& camera);
    void RenderNodes(bool deformed, Beams::Model model, std::vector<size_t>& selected);
    //void PickNodes_ForElement(const Camera3D& camera, size_t& pickingNodesForElems, const std::vector<Beams::Node>& nodes, std::vector<size_t>& selectedNodes, Beams::Model& model);
    bool PickNode(const Camera3D& camera, const Beams::NodeContainer& nodes, std::vector<size_t>& selectedNodes, Beams::Model& model, bool deformed);
    void RenderElements(const std::vector<Beams::vBeam>& elements, const Beams::NodeContainer& nodes, bool deformed, Beams::Model& model, int nodeSectionEl, std::vector<size_t>& selectedElems);
    void drawGuiActions(uint32_t& ActionFlags, const size_t& nodeSize, const Beams::NodeContainer& nodes, bool& deformed, Beams::Model& model, const Camera3D& camera, std::vector<size_t>& infoNodes, const std::vector<Beams::vBeam>& elements, std::vector<size_t>& infoElems, std::vector<size_t>& selectedNodes, std::vector<size_t>& selectedElems);
    void getGuiActionsInput(int& nodeSectionEl, uint32_t& ActionFlags, Beams::Model& model, std::vector<size_t>& infoNodes, std::vector<size_t>& selectedNodes, std::vector<size_t>& infoElems, std::vector<size_t>& selectedElems);
    bool PickElem(const Camera3D& camera, const std::vector<Beams::vBeam>& elems, std::vector<size_t>& selectedElems, Beams::Model& model);

    enum GuiFlags {
        DROPDOWN_EDIT = 1 << 0,
        NODE_ADD_ACTIVE = 1 << 1,
        NODE_REMOVE_ACTIVE = 1 << 2,
        EL_ADD_ACTIVE = 1 << 3,
        EL_REMOVE_ACTIVE = 1 << 4,
        SHOW_NODE_INFO = 1 << 5,
        SHOW_ELEM_INFO = 1 << 6,
        SECTION_WINDOW = 1 << 7,
        FORCE_ADD_ACTIVE = 1 << 8,
        FORCE_REMOVE_ACTIVE = 1 << 9,
        SHOW_FORCE_INFO = 1 << 10,
        BC_ADD_ACTIVE = 1 << 11,
        BC_REMOVE_ACTIVE = 1 << 12,
        SHOW_BC_INFO = 1 << 13

    };

    //TODO: Element Remove Crash

    //TODO: Deformed Nodes calculation once and updates when solving. 


    void runViewer(Beams::Model& model) {
        Camera3D camera;
        Vector3 cubePosition = { 0.0f, 0.0f, 0.0f };
        Vector3 rotCenter{ 0.0f,0.0f,0.0f };
        BoundingBox objectsBoundBox{ Vector3{-1.0f,-1.0f,-1.0f},{1.0f,1.0f,1.0f} };

        SetupScene(camera);


        while (!WindowShouldClose())		// run the loop untill the user presses ESCAPE or presses the Close button on the window
        {
            static std::vector<size_t> selectedNodes;
            static std::vector<size_t> selectedElems;
            static std::vector<size_t> infoNodes;
            static std::vector<size_t> infoElems;


            static size_t pickingNodesForElems = 0;
            auto& elements = model.getElements();
            static bool deformed = false;


            static int nodeSectionEl = 0;//Which action section (for nodes, elements, sections etc) is currently active
            static uint32_t ActionFlags = 0; //Which action are we curently doing


            CameraControls(rotCenter, camera, objectsBoundBox);

            const Beams::NodeContainer& nodes = model.getNodes();
            size_t nodeSize = nodes.size();


            if (IsMouseButtonReleased(MOUSE_BUTTON_MIDDLE)) {
                Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
                //for (size_t i = 0; i < nodeSize; i++) {
                for (auto& notDeletedIt : nodes) {
                    const Beams::Node& node = notDeletedIt;
                    Vector3 nodeCenter{ node.xRender,node.yRender,node.zRender };
                    RayCollision collision = GetRayCollisionSphere(ray, nodeCenter, 0.15f);
                    if (collision.hit) {
                        camera.target = nodeCenter;
                    }
                }
            }

            if (IsKeyReleased(KEY_SPACE)) {
                deformed = !deformed;
            }
            if (!model.isSolved()) deformed = false;


            bool nodePicking = ActionFlags &
                (GuiFlags::NODE_REMOVE_ACTIVE | GuiFlags::EL_ADD_ACTIVE | GuiFlags::SHOW_NODE_INFO | GuiFlags::FORCE_ADD_ACTIVE | GuiFlags::FORCE_REMOVE_ACTIVE
                    | GuiFlags::BC_ADD_ACTIVE | GuiFlags::BC_REMOVE_ACTIVE);
            if (nodePicking) {
                PickNode(camera, nodes, selectedNodes, model, deformed);
            }
            else selectedNodes.clear();


            bool elemPicking = ActionFlags & (GuiFlags::EL_REMOVE_ACTIVE | GuiFlags::SHOW_ELEM_INFO);
            if (elemPicking) {
                PickElem(camera, elements, selectedElems, model);
            }
            else selectedElems.clear();

            // drawing
            BeginDrawing();
            // Setup the back buffer for drawing (clear color and depth buffers)
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);


            RenderElements(elements, nodes, deformed, model, nodeSectionEl, selectedElems);

            RenderNodes(deformed, model, selectedNodes);

            EndMode3D();

            //TODO: Struct for moving stuff in draw functions.
            drawGuiActions(ActionFlags, nodeSize, nodes, deformed, model, camera, infoNodes, elements, infoElems, selectedNodes, selectedElems);

            getGuiActionsInput(nodeSectionEl, ActionFlags, model, infoNodes, selectedNodes, infoElems, selectedElems);


            // end the frame and get ready for the next one  (display frame, poll input, etc...)
            EndDrawing();







        }

        // cleanup

        // destroy the window and cleanup the OpenGL context
        CloseWindow();
        return;
    }

    void getGuiActionsInput(int& nodeSectionEl, uint32_t& ActionFlags, Beams::Model& model, std::vector<size_t>& infoNodes, std::vector<size_t>& selectedNodes, std::vector<size_t>& infoElems, std::vector<size_t>& selectedElems)
    {
        GuiLabel(Rectangle{ 12, 10, 140, 24 }, "Control:");
        if (GuiDropdownBox(Rectangle{ 12, 8 + 24, 140, 28 }, "NODES;ELEMENTS;SECTIONS;FORCES;BCs", &nodeSectionEl, ActionFlags & GuiFlags::DROPDOWN_EDIT)) ActionFlags ^= GuiFlags::DROPDOWN_EDIT;
        if (GuiButton(Rectangle{ 12, 700, 140, 28 }, "Solve!")) {
            model.solve();
        }
        if (nodeSectionEl == 0) {
            if (GuiButton(Rectangle{ 12 + 140 + 12, 8 + 24, 50, 28 }, "ADD")) {
                ActionFlags = GuiFlags::NODE_ADD_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62, 8 + 24, 50, 28 }, "REMOVE")) {
                ActionFlags = GuiFlags::NODE_REMOVE_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62 + 62, 8 + 24, 50, 28 }, "INFO")) {
                ActionFlags = GuiFlags::SHOW_NODE_INFO;
                for (auto infoPos : infoNodes) {
                    selectedNodes.push_back(infoPos);
                }
            }

        }
        else if (nodeSectionEl == 1) {
            if (GuiButton(Rectangle{ 12 + 140 + 12, 8 + 24, 50, 28 }, "ADD")) {
                ActionFlags = GuiFlags::EL_ADD_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62, 8 + 24, 50, 28 }, "REMOVE")) {
                ActionFlags = GuiFlags::EL_REMOVE_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62 + 62, 8 + 24, 50, 28 }, "INFO")) {
                ActionFlags = GuiFlags::SHOW_ELEM_INFO;
                for (auto infoPos : infoElems) {
                    selectedElems.push_back(infoPos);
                }
            }
        }
        else if (nodeSectionEl == 2) {
            if (GuiButton(Rectangle{ 12 + 140 + 12, 8 + 24, 50, 28 }, "MANAGE")) {
                ActionFlags = GuiFlags::SECTION_WINDOW;
            }

        }

        else if (nodeSectionEl == 3) {
            if (GuiButton(Rectangle{ 12 + 140 + 12, 8 + 24, 50, 28 }, "ADD")) {
                ActionFlags = GuiFlags::FORCE_ADD_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62, 8 + 24, 50, 28 }, "REMOVE")) {
                ActionFlags = GuiFlags::FORCE_REMOVE_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62 + 62, 8 + 24, 50, 28 }, "INFO")) {
                ActionFlags = GuiFlags::SHOW_FORCE_INFO;
            }
        }
        else if (nodeSectionEl == 4) {
            if (GuiButton(Rectangle{ 12 + 140 + 12, 8 + 24, 50, 28 }, "ADD")) {
                ActionFlags = GuiFlags::BC_ADD_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62, 8 + 24, 50, 28 }, "REMOVE")) {
                ActionFlags = GuiFlags::BC_REMOVE_ACTIVE;
            }

            if (GuiButton(Rectangle{ 24 + 140 + 62 + 62, 8 + 24, 50, 28 }, "INFO")) {
                ActionFlags = GuiFlags::SHOW_BC_INFO;
            }
        }

        GuiUnlock();
    }

    void drawGuiActions(uint32_t& ActionFlags, const size_t& nodeSize, const Beams::NodeContainer& nodes, bool& deformed, Beams::Model& model, const Camera3D& camera, std::vector<size_t>& infoNodes, const std::vector<Beams::vBeam>& elements, std::vector<size_t>& infoElems, std::vector<size_t>& selectedNodes, std::vector<size_t>& selectedElems)
    {
        if (GuiFlags::SHOW_NODE_INFO & ActionFlags) {
            for (auto& notDeleted : nodes) {
                const Beams::Node& node = notDeleted;
                Vector3 nodeCoords{ node.xRender,node.yRender,node.zRender };

                if (deformed) {

                    nodeCoords += model.getDeflectionRender(node.matrixPos);
                }

                Vector2 nodeScreenPos = GetWorldToScreen(nodeCoords, camera);
                char a[20];
                sprintf_s(a, "ID %d", (int)node.matrixPos);
                DrawText(a, (int)nodeScreenPos.x - MeasureText(a, 20) / 2, (int)nodeScreenPos.y, 20, BLACK);
            }
        }
        else {
            for (auto infoPos : infoNodes) {
                const Beams::Node& node = nodes.get_byPos(infoPos);
                Vector3 nodeCoords{ node.xRender,node.yRender,node.zRender };
                Vector3 deforms = model.getDeflectionRender(node.matrixPos);
                if (deformed) {
                    nodeCoords += deforms;
                }
                Vector3 deformsCorrect = model.getDeflection(node.matrixPos);
                Vector2 nodeScreenPos = GetWorldToScreen(nodeCoords, camera);
                char a[64];
                sprintf_s(a, "ID %d\nDeformations:\nX: %f\nY: %f\nZ: %f", (int)node.matrixPos, deformsCorrect.x, deformsCorrect.y, deformsCorrect.z);
                DrawText(a, (int)nodeScreenPos.x - MeasureText(a, 20) / 2, (int)nodeScreenPos.y, 20, BLACK);
            }
        }

        if (GuiFlags::SHOW_ELEM_INFO & ActionFlags) {
            for (auto& element : elements) {
                const Beams::Node& node1 = nodes.get_byPos(element.node1Pos);
                const Beams::Node& node2 = nodes.get_byPos(element.node2Pos);


                Vector3 n1{ node1.xRender,node1.yRender,node1.zRender };
                Vector3 n2{ node2.xRender,node2.yRender,node2.zRender };
                if (deformed) {
                    n1 += model.getDeflectionRender(node1.matrixPos);
                    n2 += model.getDeflectionRender(node2.matrixPos);
                }
                Vector2 elScreenPos = GetWorldToScreen(Vector3Add(n1, Vector3Subtract(n2, n1) / 2), camera);
                char a[20];
                sprintf_s(a, "NIDs %d | %d", (int)node1.matrixPos, (int)node2.matrixPos);
                DrawText(a, (int)elScreenPos.x - MeasureText(a, 20) / 2, (int)elScreenPos.y, 20, BLUE);
            }
        }
        else {
            for (auto infoPos : infoElems) {
                const Beams::Node& node1 = nodes.get_byPos(elements[infoPos].node1Pos);
                const Beams::Node& node2 = nodes.get_byPos(elements[infoPos].node2Pos);


                Vector3 n1{ node1.xRender,node1.yRender,node1.zRender };
                Vector3 n2{ node2.xRender,node2.yRender,node2.zRender };
                if (deformed) {
                    n1 += model.getDeflectionRender(node1.matrixPos);
                    n2 += model.getDeflectionRender(node2.matrixPos);
                }
                Vector2 elScreenPos = GetWorldToScreen(Vector3Add(n1, Vector3Subtract(n2, n1) / 2), camera);
                char a[20];
                sprintf_s(a, "NIDs %d | %d", (int)node1.matrixPos, (int)node2.matrixPos);
                DrawText(a, (int)elScreenPos.x - MeasureText(a, 20) / 2, (int)elScreenPos.y, 20, BLUE);
            }
        }

        if (GuiFlags::DROPDOWN_EDIT & ActionFlags) GuiLock();

        if (GuiFlags::NODE_ADD_ACTIVE & ActionFlags)
        {
            static int xNew;
            static int yNew;
            static int zNew;
            static bool xActive = false, yActive = false, zActive = false;
            static bool xNeg = false;
            static bool yNeg = false;
            static bool zNeg = false;
            static bool spin = false;

            if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "AddNode")) ActionFlags &= ~GuiFlags::NODE_ADD_ACTIVE;
            if (GuiValueBox(Rectangle{ 444, 56 - 13, 56, 24 }, "X", &xNew, -10000, 10000, xActive)) xActive = !xActive;
            if (GuiValueBox(Rectangle{ 444, 88 - 13, 56, 24 }, "Y", &yNew, -10000, 10000, yActive)) yActive = !yActive;
            if (GuiValueBox(Rectangle{ 444, 120 - 13, 56, 24 }, "Z", &zNew, -10000, 10000, zActive)) zActive = !zActive;
            GuiCheckBox(Rectangle{ 516, 56 - 13, 24, 24 }, "Neg", &xNeg);
            GuiCheckBox(Rectangle{ 516, 88 - 13, 24, 24 }, "Neg", &yNeg);
            GuiCheckBox(Rectangle{ 516, 120 - 13, 24, 24 }, "Neg", &zNeg);

            if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "ADD NODE!")) {
                //NodeAddActive = false;
                xNew = (xNeg) ? -xNew : xNew;
                yNew = (yNeg) ? -yNew : yNew;
                zNew = (zNeg) ? -zNew : zNew;
                model.addNode(Vector3{ (float)xNew,(float)yNew,(float)zNew });
            }
        }

        if (GuiFlags::NODE_REMOVE_ACTIVE & ActionFlags) {
            {

                static bool toggleActive = false;
                if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "RemoveNode")) ActionFlags &= ~GuiFlags::NODE_REMOVE_ACTIVE;
                GuiLabel(Rectangle{ 436, 40 , 400, 56 }, "Pick nodes with left click. Unpick with right click.\nClick CLEAR to clear selection.\nClick REMOVE to remove selected.\nNodes also remove Elements.");

                if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "REMOVE")) {
                    std::sort(selectedNodes.rbegin(), selectedNodes.rend());
                    for (size_t selected : selectedNodes) {
                        model.removeNode(selected);
                    }
                    selectedNodes.clear();
                };

                if (GuiButton(Rectangle{ 440, 112, 120, 24 }, "CLEAR")) {
                    selectedNodes.clear();
                }
            }
        }

        if (GuiFlags::SHOW_NODE_INFO & ActionFlags) {
            {
                if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Node Information")) ActionFlags &= ~GuiFlags::SHOW_NODE_INFO;
                GuiLabel(Rectangle{ 436, 40 , 400, 56 }, "Pick nodes with left click. Unpick with right click.\nClick CLEAR to clear selection.\nClick SELECT to view info for selected Nodes.\nThe info persists unless the nodes are unpicked or cleared.");
                if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "SELECT")) {
                    for (size_t selected : selectedNodes) {
                        infoNodes.push_back(selected);
                    }
                    selectedNodes.clear();
                    ActionFlags &= ~GuiFlags::SHOW_NODE_INFO;
                };
                if (GuiButton(Rectangle{ 440, 112, 120, 24 }, "CLEAR")) {
                    selectedNodes.clear();
                    infoNodes.clear();
                }
            }
        }

        if (GuiFlags::EL_ADD_ACTIVE & ActionFlags) {

            //DrawText("Pick Nodes While Holding Shift", 400, 200, 20, RED);
            static int sectionId = 0;
            static int ListViewTop = 0;
            auto& sections = model.getSections();

            std::string listNames;

            for (auto section : sections) {
                listNames += std::to_string(section.first);
                listNames += ";";
            }
            if (listNames.size() > 0) listNames.erase(listNames.size() - 1);


            if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Create Element")) ActionFlags &= ~GuiFlags::EL_ADD_ACTIVE;
            GuiListView(Rectangle{ 432, 40, 120, 96 }, listNames.data(), &ListViewTop, &sectionId);
            //std::string info = "Pick 3 nodes and the corresponding section.";
            GuiLabel(Rectangle{ 576, 40, 250, 40 }, "Pick 3 nodes and the corresponding section.\n Section properties can be managed in \"Sections\" from the dropdown");
            if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "ADD ELEMENT") && selectedNodes.size() == 3) {
                //NodeAddActive = false;
                model.addElement(selectedNodes[0], selectedNodes[1], selectedNodes[2], static_cast<size_t>(sectionId));
                selectedNodes.clear();
            }



            if (selectedNodes.size() > 3) selectedNodes.pop_back();


        }

        if (GuiFlags::EL_REMOVE_ACTIVE & ActionFlags) {
            {
                if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "RemoveElement")) ActionFlags &= ~GuiFlags::EL_REMOVE_ACTIVE;
                GuiLabel(Rectangle{ 436, 40 , 400, 40 }, "Pick elements with left click. Unpick with right click.\nClick CLEAR to clear selection.\nClick Remove to remove selected.");
                if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "REMOVE")) {
                    std::sort(selectedElems.rbegin(), selectedElems.rend());//reverse sort
                    for (size_t selected : selectedElems) {
                        model.removeElement(selected);
                    }
                    selectedElems.clear();

                };
                if (GuiButton(Rectangle{ 440, 112, 120, 24 }, "CLEAR")) {
                    selectedElems.clear();
                }
                //GuiToggle(Rectangle{ 440, 112, 120, 24 }, "TOGGLE PICK", &pickElemsActive);
            }
        }

        if (GuiFlags::SHOW_ELEM_INFO & ActionFlags) {
            {

                if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Element Information")) ActionFlags &= ~GuiFlags::SHOW_ELEM_INFO;
                if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "Select")) {
                    for (size_t selected : selectedElems) {
                        infoElems.push_back(selected);
                    }
                    selectedElems.clear();
                    ActionFlags &= ~GuiFlags::SHOW_ELEM_INFO;

                };
                if (GuiButton(Rectangle{ 440, 112, 120, 24 }, "Clear")) {
                    selectedElems.clear();
                    infoElems.clear();
                }
            }
        }

        if (GuiFlags::SECTION_WINDOW & ActionFlags) {
            static int A = 0;
            static int Ixx = 0;
            static int Iyy = 0;
            static int Izz = 0;
            static int E = 0;
            static int G = 0;
            static bool A_Active = false;
            static bool Ixx_Active = false;
            static bool Iyy_Active = false;
            static bool Izz_Active = false;
            static bool E_Active = false;
            static bool G_Active = false;

            static int prevSectionId = -1;
            static int sectionId = 0;
            static int ListViewTop = 0;
            auto& sections = model.getSections();
            bool ReadSectionInfo = prevSectionId == sectionId;
            if (!ReadSectionInfo) prevSectionId = sectionId;
            if (!ReadSectionInfo && sections.find(static_cast<int>(sectionId)) != sections.end()) {
                A = sections.find(static_cast<size_t>(sectionId))->second.Area;
                Ixx = sections.find(static_cast<size_t>(sectionId))->second.Ixx;
                Iyy = sections.find(static_cast<size_t>(sectionId))->second.Iyy;
                Izz = sections.find(static_cast<size_t>(sectionId))->second.Izz;
                E = sections.find(static_cast<size_t>(sectionId))->second.Modulus;
                G = sections.find(static_cast<size_t>(sectionId))->second.G;

            }





            std::string listNames;
            for (auto section : sections) {
                listNames += std::to_string(section.first);
                listNames += ";";
            }
            if (listNames.size() > 0) listNames.erase(listNames.size() - 1);


            if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Sections")) ActionFlags &= ~GuiFlags::SECTION_WINDOW;
            GuiListView(Rectangle{ 432, 40, 120, 96 }, listNames.data(), &ListViewTop, &sectionId);
            if (GuiValueBox(Rectangle{ 576, 40, 56, 24 }, "Area", &A, 0, 1000000, A_Active)) A_Active = !A_Active;
            if (GuiValueBox(Rectangle{ 576, 76, 56, 24 }, "Iyy", &Iyy, 0, 1000000, Iyy_Active)) Iyy_Active = !Iyy_Active;
            if (GuiValueBox(Rectangle{ 576, 112, 56, 24 }, "E", &E, 0, 1000000, E_Active)) E_Active = !E_Active;
            if (GuiValueBox(Rectangle{ 656, 40, 56, 24 }, "Ixx", &Ixx, 0, 1000000, Ixx_Active)) Ixx_Active = !Ixx_Active;
            if (GuiValueBox(Rectangle{ 656, 76, 56, 24 }, "Izz", &Izz, 0, 1000000, Izz_Active)) Izz_Active = !Izz_Active;
            if (GuiValueBox(Rectangle{ 656, 112, 56, 24 }, "G", &G, 0, 1000000, G_Active)) G_Active = !G_Active;
            if (GuiButton(Rectangle{ 728, 76, 64, 24 }, "ADD")) {
                model.addSection(A, E, G, Ixx, Iyy, Izz);

            }
            /*if (GuiButton(Rectangle{ 728, 112, 64, 24 }, "SAVE") && sections.size() > 0) {
                model.modifySection(sectionId, A, E, G, Ixx, Iyy, Izz);
            }*/


        }

        if (GuiFlags::FORCE_ADD_ACTIVE & ActionFlags)
        {
            static int xNew;
            static int yNew;
            static int zNew;
            static bool xActive = false, yActive = false, zActive = false;
            static bool xNeg = false;
            static bool yNeg = false;
            static bool zNeg = false;
            deformed = false;

            if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Add Force")) ActionFlags &= ~GuiFlags::FORCE_ADD_ACTIVE;
            if (GuiValueBox(Rectangle{ 444, 56 - 13, 56, 24 }, "X", &xNew, -10000, 10000, xActive)) xActive = !xActive;
            if (GuiValueBox(Rectangle{ 444, 88 - 13, 56, 24 }, "Y", &yNew, -10000, 10000, yActive)) yActive = !yActive;
            if (GuiValueBox(Rectangle{ 444, 120 - 13, 56, 24 }, "Z", &zNew, -10000, 10000, zActive)) zActive = !zActive;
            GuiCheckBox(Rectangle{ 516, 56 - 13, 24, 24 }, "Neg", &xNeg);
            GuiCheckBox(Rectangle{ 516, 88 - 13, 24, 24 }, "Neg", &yNeg);
            GuiCheckBox(Rectangle{ 516, 120 - 13, 24, 24 }, "Neg", &zNeg);
            GuiLabel(Rectangle{ 580, 40, 350, 56 }, "Pick Nodes and insert Force vector coordinates\nToggle NEG for Negative Values\nADD FORCE adds the force to slected Nodes\nCLEAR clears selected Nodes");
            if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "ADD FORCE") && selectedNodes.size() > 0) {
                for (auto nodePos : selectedNodes) {
                    xNew = (xNeg) ? -xNew : xNew;
                    yNew = (yNeg) ? -yNew : yNew;
                    zNew = (zNeg) ? -zNew : zNew;
                    model.addForce(nodePos, 0, xNew);
                    model.addForce(nodePos, 1, yNew);
                    model.addForce(nodePos, 2, zNew);

                }
            }
            if (GuiButton(Rectangle{ 580, 112, 120, 24 }, "CLEAR")) {
                selectedNodes.clear();
            }

        }

        if (GuiFlags::FORCE_REMOVE_ACTIVE & ActionFlags) {
            {

                static bool toggleActive = false;
                if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Remove Force")) ActionFlags &= ~GuiFlags::FORCE_REMOVE_ACTIVE;
                GuiLabel(Rectangle{ 436, 40 , 400, 56 }, "Pick nodes with left click. Unpick with right click.\nClick CLEAR to clear selection.\nClick REMOVE to remove selected Forces.\n ");

                if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "REMOVE")) {
                    for (size_t selected : selectedNodes) {
                        model.removeForce(selected);
                    }
                    selectedNodes.clear();
                };

                if (GuiButton(Rectangle{ 440, 112, 120, 24 }, "CLEAR")) {
                    selectedNodes.clear();
                }
            }
        }

        if (GuiFlags::SHOW_FORCE_INFO & ActionFlags) {
            {
                for (auto& force : model.getForces()) {
                    const Beams::Node& node = nodes.get_byPos(force.first);
                    Vector3 nodeCoords{ node.xRender,node.yRender,node.zRender };

                    if (deformed) {

                        nodeCoords += model.getDeflectionRender(node.matrixPos);
                    }

                    Vector2 nodeScreenPos = GetWorldToScreen(nodeCoords, camera);
                    char a[64];
                    sprintf_s(a, "Force:\nX: %f\nY: %f\nZ: %f", force.second[0], force.second[1], force.second[2]);
                    DrawText(a, (int)nodeScreenPos.x - MeasureText(a, 20) / 2, (int)nodeScreenPos.y, 20, BLACK);
                }
            }
        }


        if (GuiFlags::BC_ADD_ACTIVE & ActionFlags)
        {

            if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Add Force")) ActionFlags &= ~GuiFlags::BC_ADD_ACTIVE;

            GuiLabel(Rectangle{ 580, 40, 350, 56 }, "Pick Nodes with left click, unpick with right click\nADD BC adds the BC to selcted Nodes\nCLEAR clears selected Nodes");
            if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "ADD BC") && selectedNodes.size() > 0) {
                for (auto nodePos : selectedNodes) {
                    model.addBCfixed(nodePos);
                    selectedNodes.clear();
                }
            }
            if (GuiButton(Rectangle{ 580, 112, 120, 24 }, "CLEAR")) {
                selectedNodes.clear();
            }

        }

        if (GuiFlags::BC_REMOVE_ACTIVE & ActionFlags) {
            {

                static bool toggleActive = false;
                if (GuiWindowBox(Rectangle{ 424, 8, 448, 136 }, "Remove Force")) ActionFlags &= ~GuiFlags::BC_REMOVE_ACTIVE;
                GuiLabel(Rectangle{ 436, 40 , 400, 56 }, "Pick nodes with left click. Unpick with right click.\nClick CLEAR to clear selection.\nClick REMOVE to remove selected Forces.\n ");

                if (GuiButton(Rectangle{ 736, 112, 120, 24 }, "REMOVE")) {
                    for (size_t selected : selectedNodes) {
                        model.removeBCfixed(selected);
                    }
                    selectedNodes.clear();
                };

                if (GuiButton(Rectangle{ 440, 112, 120, 24 }, "CLEAR")) {
                    selectedNodes.clear();
                }
            }
        }

        if (GuiFlags::SHOW_BC_INFO & ActionFlags) {
            {
                for (auto& bc : model.getBCfixed()) {
                    const Beams::Node& node = nodes.get_byPos(bc);
                    Vector3 nodeCoords{ node.xRender,node.yRender,node.zRender };

                    if (deformed) {

                        nodeCoords += model.getDeflectionRender(node.matrixPos);
                    }

                    Vector2 nodeScreenPos = GetWorldToScreen(nodeCoords, camera);
                    char a[64];
                    sprintf_s(a, "Fixed");
                    DrawText(a, (int)nodeScreenPos.x - MeasureText(a, 20) / 2, (int)nodeScreenPos.y, 20, BLACK);
                }
            }
        }

    }

    void RenderElements(const std::vector<Beams::vBeam>& elements, const Beams::NodeContainer& nodes, bool deformed, Beams::Model& model, int nodeSectionEl, std::vector<size_t>& selectedElems)
    {
        for (auto& element : elements) {
            const Beams::Node& node1 = nodes.get_byPos(element.node1Pos);
            const Beams::Node& node2 = nodes.get_byPos(element.node2Pos);

            Vector3 n1{ node1.xRender,node1.yRender,node1.zRender };
            Vector3 n2{ node2.xRender,node2.yRender,node2.zRender };

            if (deformed) {
                n1 += model.getDeflectionRender(node1.matrixPos);
                n2 += model.getDeflectionRender(node2.matrixPos);
            }

            if (nodeSectionEl == 1) DrawSphere(Vector3Add(n1, (Vector3Subtract(n2, n1) / 2)), 0.15f, BLUE);
            DrawCapsule(n1, n2, 0.1f, 2, 2, BLUE);
        }

        for (size_t pos : selectedElems) {
            const Beams::Node& node1 = nodes.get_byPos(elements[pos].node1Pos);
            const Beams::Node& node2 = nodes.get_byPos(elements[pos].node2Pos);

            Vector3 n1{ node1.xRender,node1.yRender,node1.zRender };
            Vector3 n2{ node2.xRender,node2.yRender,node2.zRender };
            DrawSphereWires(Vector3Add(n1, (Vector3Subtract(n2, n1) / 2)), 0.2, 5, 5, DARKGREEN);
        }
    }

    bool PickNode(const Camera3D& camera, const Beams::NodeContainer& nodes, std::vector<size_t>& selectedNodes, Beams::Model& model, bool deformed) {
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            RayCollision collision;

            for (auto& notDeleted : nodes) {
                const Beams::Node& node = notDeleted;
                Vector3 nodeCenter{ node.xRender,node.yRender,node.zRender };
                if (deformed) {
                    nodeCenter += model.getDeflectionRender(node.matrixPos);
                }

                collision = GetRayCollisionSphere(ray, nodeCenter, 0.3f);
                if (collision.hit) {
                    selectedNodes.push_back(node.pos);
                    return true;
                }
            }
        }
        else if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            RayCollision collision;

            for (auto& notDeleted : nodes) {
                const Beams::Node& node = notDeleted;
                Vector3 nodeCenter{ node.xRender,node.yRender,node.zRender };
                if (deformed) {
                    nodeCenter += model.getDeflectionRender(node.matrixPos);
                }

                collision = GetRayCollisionSphere(ray, nodeCenter, 0.3f);
                if (collision.hit) {
                    auto it = std::find(selectedNodes.begin(), selectedNodes.end(), node.pos);
                    if (it != selectedNodes.end()) selectedNodes.erase(it);
                    return true;
                }
            }
        }
        return false;
    }

    bool PickElem(const Camera3D& camera, const std::vector<Beams::vBeam>& elements, std::vector<size_t>& selectedElems, Beams::Model& model) {
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            RayCollision collision;

            for (size_t i = 0; i < elements.size(); i++) {
                const Beams::Node& node1 = model.getNodes().get_byPos(elements[i].node1Pos);
                const Beams::Node& node2 = model.getNodes().get_byPos(elements[i].node2Pos);



                Vector3 n1{ node1.xRender,node1.yRender,node1.zRender };
                Vector3 n2{ node2.xRender,node2.yRender,node2.zRender };


                Vector3 elemCenter = Vector3Add(n1, (Vector3Subtract(n2, n1) / 2));
                collision = GetRayCollisionSphere(ray, elemCenter, 0.3f);
                if (collision.hit) {
                    selectedElems.push_back(i);
                    return true;
                }
            }
        }
        else if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT)) {
            Ray ray = GetScreenToWorldRay(GetMousePosition(), camera);
            RayCollision collision;

            for (size_t i = 0; i < elements.size(); i++) {
                const Beams::Node& node1 = model.getNodes().get_byPos(elements[i].node1Pos);
                const Beams::Node& node2 = model.getNodes().get_byPos(elements[i].node2Pos);



                Vector3 n1{ node1.xRender,node1.yRender,node1.zRender };
                Vector3 n2{ node2.xRender,node2.yRender,node2.zRender };

                Vector3 elemCenter = Vector3Add(n1, (Vector3Subtract(n2, n1) / 2));
                collision = GetRayCollisionSphere(ray, elemCenter, 0.3f);
                if (collision.hit) {
                    auto it = std::find(selectedElems.begin(), selectedElems.end(), i);
                    if (it != selectedElems.end()) selectedElems.erase(it);
                    return true;
                }
            }
        }
        return false;
    }

    void RenderNodes(bool deformed, Beams::Model model, std::vector<size_t>& selected)
    {
        const Beams::NodeContainer& nodes = model.getNodes();

        size_t nodeSize = nodes.size();

        for (auto& notDeleted : nodes) {
            const Beams::Node& node = notDeleted;
            Vector3 nodeCoords{ node.xRender ,node.yRender ,node.zRender };


            if (deformed) {
                nodeCoords += model.getDeflectionRender(node.matrixPos);
            }
            DrawSphere(nodeCoords, 0.15f, RED);
        }

        for (auto forceIt : model.getForces()) {
            const Beams::Node& node = nodes.get_byPos(forceIt.first);
            Vector3 nodeCoords{ node.xRender ,node.yRender ,node.zRender };
            if (deformed) {
                nodeCoords += model.getDeflectionRender(node.matrixPos);
            }


            Vector3 force{ forceIt.second[0],forceIt.second[1],forceIt.second[2] };
            if (std::abs(force.x) + std::abs(force.y) + std::abs(force.z) > 0.001) {
                force = Vector3Normalize(force);
                Vector3 forceEndpoint = Vector3Add(nodeCoords, force);
                DrawLine3D(nodeCoords, forceEndpoint, DARKBROWN);
                DrawCylinderEx(Vector3Add(nodeCoords, force / 2), forceEndpoint, 0.1f, 0.0f, 10, DARKBROWN);
            }
        }

        for (auto BC : model.getBCfixed()) {
            const Beams::Node& node = nodes.get_byPos(BC);
            Vector3 nodeCoords{ node.xRender ,node.yRender ,node.zRender };
            DrawCubeWires(nodeCoords, 1.0f, 1.0f, 1.0f, MAROON);
        }

        for (size_t pos : selected) {
            const Beams::Node& node = nodes.get_byPos(pos);
            Vector3 nodeCoords{ node.xRender ,node.yRender ,node.zRender };
            if (deformed) {
                nodeCoords += model.getDeflectionRender(node.matrixPos);
            }
            DrawSphereWires(nodeCoords, 0.2, 5, 5, DARKGREEN);
        }

        //DrawCube(Vector3Zero(), 2.0f, 2.0f, 2.0f, RED);
        DrawGrid(10, 10.0f);
    }

    void SetupScene(Camera3D& camera)
    {
        const int screenWidth = 1280;
        const int screenHeight = 800;

        // Camera position
        camera.position.x = -100.0f;
        camera.position.y = 120.0f;
        camera.position.z = -100.0f;

        //camera target
        camera.target.x = 0.0f;
        camera.target.y = 0.0f;
        camera.target.z = 0.0f;

        camera.up.x = 0.0f;
        camera.up.y = 1.0f;
        camera.up.z = 0.0f;// Camera up vector (rotation towards target)

        camera.fovy = 5.0f;                                    // Camera field-of-view Y
        camera.projection = CAMERA_PERSPECTIVE;




        SetTargetFPS(60);               // Set our game to run at 60 frames-per-second

        // Tell the window to use vsync and work on high DPI displays
        //SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
        SetConfigFlags(FLAG_WINDOW_HIGHDPI);


        // Create the window and OpenGL context
        InitWindow(screenWidth, screenHeight, "VBeams");
    }

    void CameraControls(Vector3& rotCenter, Camera3D& camera, const BoundingBox& objectsBoundBox)
    {

        if (IsKeyPressed(KEY_Z)) {
            rotCenter = Vector3Zero();
            camera.target = rotCenter;
        }
        if (IsKeyDown(KEY_LEFT_CONTROL) && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            UpdateCameraProAroundTarget(&camera, Vector3{ 0,-GetMouseDelta().x * 0.03f,GetMouseDelta().y * 0.03f}, Vector3{0,0,0}, 0.0f);
           
        }
        else if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {

            //UpdateCamera(&camera, CAMERA_THIRD_PERSON);
            //camera.target = Vector3{ 0.0f,0.0f, 0.0f };
            CameraRotateAround(&camera, camera.target, GetMouseDelta().x * 0.003f, GetMouseDelta().y * 0.005f);
            //         (&camera,Vector3{0,0,0},
            //	Vector3{
            //	GetMouseDelta().x * 0.3f,                            // Rotation: yaw
            //	GetMouseDelta().y * 0.3f,                            // Rotation: pitch
            //	0.0f                                                // Rotation: roll
            //},
            //GetMouseWheelMove() * 2.0f);                              // Move to target (zoom)
            SetMouseCursor(MOUSE_CURSOR_CROSSHAIR);



        }
        else {
            UpdateCameraProAroundTarget(&camera, Vector3{ 0,0,0 }, Vector3{ 0,0,0 }, GetMouseWheelMove() * 10.0f);
            SetMouseCursor(MOUSE_CURSOR_ARROW);

        }
    }
}