#include "gcs_comm.h"
#include "gcs_ui.h"
#include <GLFW/glfw3.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <memory>
#include <iostream>

// Main UI loop
int main() {
    // Try GLFW init
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    // Explicit OpenGL version for consistency
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // Resize false
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

    // Try to create window
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Ground Control System", nullptr, nullptr);

    if (!window) {
        std::cerr << "GLFW: Failed to create window\n";
        glfwTerminate();
        return -1;
    }

    // Apply window context
    glfwMakeContextCurrent(window);
    // VSync enable (for monitor refresh rate)
    glfwSwapInterval(1);

    // ImGui version check and context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // Init ImGui backend glue code (input/rendering)
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Color for clearing screen
    glClearColor(0.188f, 0.0f, 0.471f, 1.0f);

    // Unique pointer to connection (starts nullptr)
    std::unique_ptr<gcs_comm::Connection> connection;

    // While window not closed
    while (!glfwWindowShouldClose(window)) {
        // Process events
        glfwPollEvents();

        // New frame for OpenGL, GLFW and ImGui
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Draw and render ImGui mainwindow (gcs_ui)
        ui::DrawMainWindow(connection);
        ImGui::Render();

        // Retrieve pixel dimensions of the drawable area
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        // Render area
        glViewport(0, 0, display_w, display_h);

        // Wipe previous frame
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw ImGui content
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap from old frame to new frame
        glfwSwapBuffers(window);
    }

    // Shutdown for OpenGL, GLFW, ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;

}


