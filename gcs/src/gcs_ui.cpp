#include "gcs_ui.h"
#include "imgui.h"
#include <cstring>
#include "geofence.h"

namespace ui {
    // Function to draw connection panel
    void DrawConnection(std::unique_ptr<gcs_comm::Connection>& connection) {
        // Comport input buffer
        static char portBuffer[32] = "COM";

        // COM port input
        ImGui::InputText("COM Port", portBuffer, sizeof(portBuffer));

        // If no connection
        if (!connection || !connection->isOpen()) {
            // Draw connect button
            if (ImGui::Button("Connect")) {
                // Make unique pointer initializing connection on input port, assign to connection var
                connection = std::make_unique<gcs_comm::Connection>(portBuffer);
                // If connection not open, destroy connection and reset pointer
                if (!connection->isOpen()) {
                    connection.reset();
                }
            }
        } else {
            // Disconnect button to destroy connection and reset pointer
            if (ImGui::Button("Disconnect")) {
                connection.reset();
            }
        }
        // Port side + telemetry heartbeat connection check
        bool connected = connection && connection->isOpen() && connection->getState().connected;

        // Connection status
        ImGui::TextColored(
            connected ? ImVec4(0, 1, 0, 1) : ImVec4(1, 0, 0, 1),
            connected ? "Connected" : "Disconnected"
        );
    }

    // Function to draw 2D drone position display
    void DrawPosition(gcs_comm::Connection& connection) {
        // Drawing cursor absolute position
        ImVec2 canvasPos = ImGui::GetCursorScreenPos();
        // Fixed drawing area for drone position
        ImVec2 canvasSize = ImVec2(670, 670);

        // Get low-level draw command list to draw grid
        ImDrawList* drawList = ImGui::GetWindowDrawList();

        // Draw background
        drawList->AddRectFilled(canvasPos, ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y), IM_COL32(20, 20, 20, 255));

        // Construct full display bounds (geofence + 10m for geofence visibility)
        float fullMinX = config::MIN_X-10.0f, fullMaxX = config::MAX_X+10.0f;
        float fullMinY = config::MIN_Y-10.0f, fullMaxY = config::MAX_Y+10.0f;
        float fullWidth = fullMaxX - fullMinX;
        float fullHeight = fullMaxY - fullMinY;

        // Lambda function to display world coordinates to pixel position
        auto convertPx = [&](float x, float y) -> ImVec2 {
            // Calculate distance from edge (% out of 1)
            float nx = (x - fullMinX) / fullWidth;
            float ny = (y - fullMinY) / fullHeight;
            // Convert to window pixel position (Y flipped for window use)
            return ImVec2(canvasPos.x + nx * canvasSize.x, canvasPos.y + (1.0f - ny)*canvasSize.y);
        };

        // 2D grid (10m)
        // Y axis lines
        for (float i = fullMinX; i <= fullMaxX; i += 10.0f) {
            ImVec2 top = convertPx(i, fullMaxY);
            ImVec2 bottom = convertPx(i, fullMinY);
            drawList->AddLine(top, bottom, IM_COL32(60, 60, 60, 255));
        }
        // X axis lines
        for (float i = fullMinY; i <= fullMaxY; i += 10.0f) {
            ImVec2 left = convertPx(fullMinX, i);
            ImVec2 right = convertPx(fullMaxX, i);
            drawList->AddLine(left, right, IM_COL32(60, 60, 60, 255));
        }

        // Geofence
        // Left top corner
        ImVec2 leftTop = convertPx(config::MIN_X, config::MAX_Y);
        // Right bottom corner
        ImVec2 rightBottom = convertPx(config::MAX_X, config::MIN_Y);
        // Draw geofence rectangle
        drawList->AddRect(leftTop, rightBottom, IM_COL32(100, 100, 255, 255), 0.0f, 0, 2.0f);

        // Drone position
        gcs_comm::DStateGCS state = connection.getState();
        ImVec2 dronePos = convertPx(state.X, state.Y);

        // Draw circle (gray when disarmed, green when armed)
        drawList->AddCircleFilled(dronePos, 6.0f, state.armed ? IM_COL32(0, 255, 0, 255) : IM_COL32(150, 150, 150, 255));

        // Reserve display space
        ImGui::Dummy(canvasSize);
    }

    // Draw telemetry from getState()
    void DrawTelemetry(gcs_comm::Connection& connection) {
        gcs_comm::DStateGCS state = connection.getState();
        ImGui::Text("X: %.2f  Y: %.2f  Alt: %.2f", state.X, state.Y, state.alt);
        ImGui::Text(state.armed ? "ARMED" : "DISARMED");
    }

    // Draw command buttons
    void DrawCommands(gcs_comm::Connection& connection) {
        // Arm command
        if (ImGui::Button("ARM")) {
            connection.sendArm();
        }
        // Land command
        ImGui::SameLine();
        if (ImGui::Button("LAND")) {
            connection.sendLand();
        }
        // GoTo command
        static float goToX = 0.0f, goToY = 0.0f;
        ImGui::InputFloat("Target X", &goToX);
        ImGui::InputFloat("Target Y", &goToY);
        if (ImGui::Button("GO TO")) {
            connection.sendGoTo(goToX, goToY);
        }
    }

    // Draw log output window
    void DrawLog(gcs_comm::Connection& connection) {
        // Child window
        ImGui::BeginChild("Log", ImVec2(0, 680), true);
        // Get log vector, turn into text elements
        for (const auto& entry : connection.getLogs()) {
            ImGui::TextUnformatted(entry.msg.c_str());
        }
        ImGui::EndChild();
    }

    // Main window function
    void DrawMainWindow(std::unique_ptr<gcs_comm::Connection>& connection) {
        // Each element position/size is hardcoded to create simple structure and ordering
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImVec2(300, 100));
        ImGui::Begin("Connection", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
        // Connection display first
        DrawConnection(connection);
        ImGui::End();

        // If connection established, show telemetry, commands, log and 2D position
        if (connection && connection->isOpen()) {
            ImGui::SetNextWindowPos(ImVec2(0, 101));
            ImGui::SetNextWindowSize(ImVec2(300, 300));
            ImGui::Begin("Commands", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            DrawCommands(*connection);
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(0, 402));
            ImGui::SetNextWindowSize(ImVec2(300, 318));
            ImGui::Begin("Telemetry", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            DrawTelemetry(*connection);
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(301, 0));
            ImGui::SetNextWindowSize(ImVec2(680, 720));
            ImGui::Begin("Drone position", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize);
            DrawPosition(*connection);
            ImGui::End();

            ImGui::SetNextWindowPos(ImVec2(982, 0));
            ImGui::SetNextWindowSize(ImVec2(298, 720));
            ImGui::Begin("Logs");
            DrawLog(*connection);
            ImGui::End();
        }

    }
}
