#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "WaypointRenderer.hpp"
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <queue>
#include <algorithm>

const int MAP_W = 100;
const int MAP_H = 60;
bool base_walkable[MAP_W][MAP_H];

void init_map() {
    for (int x=0; x<MAP_W; ++x)
        for (int y=0; y<MAP_H; ++y)
            base_walkable[x][y] = true;
    
    // A1-E4 Racks (5 columns, 4 rows)
    int cols[] = {8, 25, 42, 59, 76};
    int rows[] = {5, 15, 25, 35};
    for(int cx : cols) {
        for(int cy : rows) {
            for(int dx=0; dx<12; ++dx) {
                for(int dy=0; dy<6; ++dy) {
                    if (cx+dx < MAP_W && cy+dy < MAP_H)
                        base_walkable[cx+dx][cy+dy] = false;
                }
            }
        }
    }
}

struct Robot {
    int id;
    float position[2];
    float battery;
    const char* status;
    float speed;
    float target_position[2];
    std::vector<ImVec2> current_path;
    float replan_timer;
    float wait_timer;

    void set_target(float tx, float ty);
    void move(float dt);
};

extern std::vector<Robot> global_robots;

struct Node {
    int x, y;
    float g, h;
    Node* parent;
    bool operator>(const Node& o) const { return (g + h) > (o.g + o.h); }
};

std::vector<ImVec2> find_path(int sx, int sy, int gx, int gy, int robot_id) {
    if (sx < 0 || sx >= MAP_W || sy < 0 || sy >= MAP_H) return {};
    if (gx < 0 || gx >= MAP_W || gy < 0 || gy >= MAP_H) return {};
    if (!base_walkable[gx][gy]) return {};

    bool walkable[MAP_W][MAP_H];
    for (int x=0; x<MAP_W; ++x)
        for (int y=0; y<MAP_H; ++y)
            walkable[x][y] = base_walkable[x][y];
            
    // Add other robots as dynamic obstacles
    for(const auto& r : global_robots) {
        if(r.id != robot_id) {
            int rx = std::round(r.position[0]);
            int ry = std::round(r.position[1]);
            if(rx>=0 && rx<MAP_W && ry>=0 && ry<MAP_H) {
                walkable[rx][ry] = false;
            }
        }
    }
    
    walkable[gx][gy] = true; 

    std::vector<Node*> all_nodes;
    auto create_node = [&](int x, int y, float g, float h, Node* p) {
        Node* n = new Node{x, y, g, h, p};
        all_nodes.push_back(n);
        return n;
    };
    
    auto comp = [](Node* a, Node* b){ return *a > *b; };
    std::priority_queue<Node*, std::vector<Node*>, decltype(comp)> open(comp);
    
    bool closed[MAP_W][MAP_H] = {false};
    float g_score[MAP_W][MAP_H];
    for(int i=0;i<MAP_W;++i) for(int j=0;j<MAP_H;++j) g_score[i][j] = 1e9f;
    
    g_score[sx][sy] = 0;
    open.push(create_node(sx, sy, 0, std::abs(gx-sx)+std::abs(gy-sy), nullptr));
    
    Node* best = nullptr;
    while(!open.empty()) {
        Node* curr = open.top();
        open.pop();
        
        if (curr->x == gx && curr->y == gy) {
            best = curr;
            break;
        }
        
        if (closed[curr->x][curr->y]) continue;
        closed[curr->x][curr->y] = true;
        
        int dx[] = {1,-1,0,0,1,1,-1,-1};
        int dy[] = {0,0,1,-1,1,-1,1,-1};
        for(int i=0; i<8; ++i) {
            int nx = curr->x + dx[i];
            int ny = curr->y + dy[i];
            
            if(nx<0 || nx>=MAP_W || ny<0 || ny>=MAP_H) continue;
            if(!walkable[nx][ny]) continue;
            
            if (i >= 4) {
                if (!walkable[curr->x][ny] || !walkable[nx][curr->y]) continue;
            }
            
            float cost = (i < 4) ? 1.0f : 1.414f;
            float new_g = curr->g + cost;
            if (new_g < g_score[nx][ny]) {
                g_score[nx][ny] = new_g;
                float h = std::abs(gx-nx) + std::abs(gy-ny);
                open.push(create_node(nx, ny, new_g, h, curr));
            }
        }
    }
    
    std::vector<ImVec2> path;
    if (best) {
        Node* curr = best;
        while(curr) {
            path.push_back(ImVec2(curr->x, curr->y));
            curr = curr->parent;
        }
        std::reverse(path.begin(), path.end());
    }
    
    for(auto n : all_nodes) delete n;
    return path;
}

void Robot::set_target(float tx, float ty) {
    target_position[0] = tx;
    target_position[1] = ty;
    current_path = find_path(std::round(position[0]), std::round(position[1]), std::round(tx), std::round(ty), id);
    replan_timer = 1.0f;
    wait_timer = 0.0f;
}

void Robot::move(float dt) {
    if (current_path.empty()) {
        int tx, ty;
        do {
            tx = rand() % MAP_W;
            ty = rand() % MAP_H;
        } while(!base_walkable[tx][ty]);
        set_target(tx, ty);
        status = "Idle";
        return;
    }
    
    ImVec2 next_wp = current_path.front();
    float dx = next_wp.x - position[0];
    float dy = next_wp.y - position[1];
    float dist = std::sqrt(dx*dx + dy*dy);
    
    // Check for potential collisions
    bool is_blocked = false;
    for(const auto& r : global_robots) {
        if (r.id != id) {
            float r_dx = r.position[0] - next_wp.x;
            float r_dy = r.position[1] - next_wp.y;
            if (std::sqrt(r_dx*r_dx + r_dy*r_dy) < 1.5f) { // Collision radius
                is_blocked = true;
                break;
            }
        }
    }
    
    if (is_blocked) {
        status = "Waiting";
        wait_timer += dt;
        if (wait_timer > 3.0f) {
            // Replan after waiting 3 seconds
            std::vector<ImVec2> new_path = find_path(std::round(position[0]), std::round(position[1]), std::round(target_position[0]), std::round(target_position[1]), id);
            if (!new_path.empty()) {
                current_path = new_path;
            }
            wait_timer = 0.0f;
        }
        return; // Do not move
    }
    
    status = "Moving";
    wait_timer = 0.0f; // Reset wait timer
    
    replan_timer -= dt;
    if (replan_timer <= 0.0f) {
        replan_timer = 0.5f;
        
        std::vector<ImVec2> new_path = find_path(std::round(position[0]), std::round(position[1]), std::round(target_position[0]), std::round(target_position[1]), id);
        if (!new_path.empty()) {
            current_path = new_path;
            next_wp = current_path.front();
            dx = next_wp.x - position[0];
            dy = next_wp.y - position[1];
            dist = std::sqrt(dx*dx + dy*dy);
        }
    }

    if (dist > 0.1f) {
        position[0] += (dx / dist) * speed * dt;
        position[1] += (dy / dist) * speed * dt;
        if (battery > 0) battery -= dt * 0.05f;
    } else {
        current_path.erase(current_path.begin());
    }
}

std::vector<Robot> global_robots;


static void glfw_error_callback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

int main(int, char**) {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(1280, 720, "Logistics Forklift Simulator", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    init_map();

    bool is_simulation_running = false;
    float simulation_speed = 1.0f;
    
    global_robots.push_back({1, {5.0f, 55.0f}, 85.0f, "Idle", 8.0f, {0,0}, {}, 0.0f, 0.0f});
    global_robots.push_back({2, {30.0f, 55.0f}, 42.0f, "Idle", 8.0f, {0,0}, {}, 0.0f, 0.0f});
    global_robots.push_back({3, {80.0f, 55.0f}, 99.0f, "Idle", 8.0f, {0,0}, {}, 0.0f, 0.0f});

    for(auto& r : global_robots) {
        r.set_target(50, 45); // Actually walkable aisle position
    }

    ImVec4 clear_color = ImVec4(0.15f, 0.15f, 0.15f, 1.00f);
    double last_time = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        double current_time = glfwGetTime();
        float dt = static_cast<float>(current_time - last_time);
        last_time = current_time;

        if (is_simulation_running) {
            for (auto& robot : global_robots) {
                robot.move(dt * simulation_speed);
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

        ImGui::Begin("DockSpace Demo", nullptr, window_flags);
        ImGui::PopStyleVar(2);
        
        ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);

        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Exit")) glfwSetWindowShouldClose(window, true);
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }
        ImGui::End();

        ImGui::Begin("Map View (Factory Floor)");
        
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
        ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
        if (canvas_sz.x < 100.0f) canvas_sz.x = 100.0f;
        if (canvas_sz.y < 100.0f) canvas_sz.y = 100.0f;
        ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);
        
        float scale_x = canvas_sz.x / MAP_W;
        float scale_y = canvas_sz.y / MAP_H;

        // Dark/Cyber theme background
        draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(15, 18, 25, 255));
        
        // Grid pattern
        for (float x = 0; x < canvas_sz.x; x += scale_x * 5.0f) {
            draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(255, 255, 255, 10));
        }
        for (float y = 0; y < canvas_sz.y; y += scale_y * 5.0f) {
            draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(255, 255, 255, 10));
        }
        
        draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(0, 255, 255, 100), 0.0f, 0, 2.0f);

        auto draw_zone = [&](int x, int y, int w, int h, ImU32 col, ImU32 border_col, const char* label) {
            ImVec2 p_min = ImVec2(canvas_p0.x + x * scale_x, canvas_p0.y + y * scale_y);
            ImVec2 p_max = ImVec2(canvas_p0.x + (x+w) * scale_x, canvas_p0.y + (y+h) * scale_y);
            // Glassmorphism effect (semi-transparent filled with rounded corners)
            draw_list->AddRectFilled(p_min, p_max, col, 8.0f);
            draw_list->AddRect(p_min, p_max, border_col, 8.0f, 0, 2.0f);
            draw_list->AddText(ImVec2(p_min.x + 10, p_min.y + 10), IM_COL32(255,255,255,255), label);
        };
        
        draw_zone(0, 48, 15, 12, IM_COL32(150, 50, 200, 40), IM_COL32(200, 100, 255, 150), "Charging");
        draw_zone(20, 48, 25, 12, IM_COL32(50, 150, 255, 40), IM_COL32(100, 200, 255, 150), "Inbound");
        draw_zone(48, 48, 24, 12, IM_COL32(50, 150, 255, 40), IM_COL32(100, 200, 255, 150), "Packing");
        draw_zone(75, 48, 25, 12, IM_COL32(50, 200, 100, 40), IM_COL32(100, 255, 150, 150), "Outbound");

        // Pseudo-3D Obstacles
        for (int x=0; x<MAP_W; ++x) {
            for (int y=0; y<MAP_H; ++y) {
                if (!base_walkable[x][y]) {
                    ImVec2 p_min = ImVec2(canvas_p0.x + x * scale_x, canvas_p0.y + y * scale_y);
                    ImVec2 p_max = ImVec2(canvas_p0.x + (x+1) * scale_x, canvas_p0.y + (y+1) * scale_y);
                    
                    // Shadow / Side face (bottom right offset)
                    draw_list->AddRectFilled(ImVec2(p_min.x+2, p_min.y+2), ImVec2(p_max.x+2, p_max.y+2), IM_COL32(20, 25, 35, 255));
                    // Top face
                    draw_list->AddRectFilled(p_min, p_max, IM_COL32(40, 50, 70, 255));
                    // Highlight edge
                    draw_list->AddLine(p_min, ImVec2(p_max.x, p_min.y), IM_COL32(80, 100, 130, 255));
                    draw_list->AddLine(p_min, ImVec2(p_min.x, p_max.y), IM_COL32(80, 100, 130, 255));
                }
            }
        }
        
        int cols[] = {8, 25, 42, 59, 76};
        int rows[] = {5, 15, 25, 35};
        const char* col_labels[] = {"A", "B", "C", "D", "E"};
        for(int i=0; i<5; ++i) {
            for(int j=0; j<4; ++j) {
                ImVec2 label_pos = ImVec2(canvas_p0.x + (cols[i]+5)*scale_x, canvas_p0.y + (rows[j]-2)*scale_y);
                std::string label = std::string(col_labels[i]) + std::to_string(j+1);
                draw_list->AddText(label_pos, IM_COL32(150,200,255,255), label.c_str());
            }
        }

        double time = glfwGetTime();
        float pulse = (sin(time * 5.0) + 1.0f) * 0.5f; // 0.0 ~ 1.0

        for (const auto& robot : global_robots) {
            // [담당 기능] 독립 모듈 WaypointRenderer 호출 (팀원 코드 건드리지 않음)
            WaypointRenderer::Render(
                draw_list,
                ImVec2(robot.position[0], robot.position[1]),
                robot.current_path,
                ImVec2(robot.target_position[0], robot.target_position[1]),
                canvas_p0, scale_x, scale_y, time
            );

            ImVec2 pos = ImVec2(canvas_p0.x + robot.position[0] * scale_x, canvas_p0.y + robot.position[1] * scale_y);

            float dx = robot.current_path.empty() ? (robot.target_position[0] - robot.position[0]) : (robot.current_path.front().x - robot.position[0]);
            float dy = robot.current_path.empty() ? (robot.target_position[1] - robot.position[1]) : (robot.current_path.front().y - robot.position[1]);
            float angle = std::atan2(dy, dx);
            float size = std::min(scale_x, scale_y) * 2.0f;
            if (size < 8.0f) size = 8.0f;

            ImVec2 p1 = ImVec2(pos.x + std::cos(angle) * size, pos.y + std::sin(angle) * size);
            ImVec2 p2 = ImVec2(pos.x + std::cos(angle + 2.6f) * size, pos.y + std::sin(angle + 2.6f) * size);
            ImVec2 p3 = ImVec2(pos.x, pos.y);
            ImVec2 p4 = ImVec2(pos.x + std::cos(angle - 2.6f) * size, pos.y + std::sin(angle - 2.6f) * size);

            // Pulsing ring effect
            draw_list->AddCircle(pos, size * 1.5f + pulse * 5.0f, IM_COL32(0, 255, 255, (int)(100 * (1.0f - pulse))), 0, 2.0f);
            
            // Robot Body (Cyan/Blue Theme)
            ImU32 robot_color = IM_COL32(0, 200, 255, 255);
            if (std::string(robot.status) == "Waiting") {
                robot_color = IM_COL32(255, 200, 0, 255); // Orange/Yellow if waiting
                draw_list->AddCircle(pos, size * 1.5f + pulse * 5.0f, IM_COL32(255, 200, 0, (int)(100 * (1.0f - pulse))), 0, 2.0f);
            }

            draw_list->AddCircleFilled(pos, size * 1.2f, IM_COL32(0, 50, 100, 100));
            draw_list->AddQuadFilled(p1, p2, p3, p4, robot_color);
            draw_list->AddQuad(p1, p2, p3, p4, IM_COL32(255, 255, 255, 255), 1.5f);

            std::string label = "R" + std::to_string(robot.id);
            draw_list->AddText(ImVec2(pos.x + 12, pos.y - 12), IM_COL32(255, 255, 255, 255), label.c_str());
        }
        ImGui::End();

        ImGui::Begin("Control Panel");
        if (is_simulation_running) {
            if (ImGui::Button("Stop Simulation", ImVec2(-1, 30))) is_simulation_running = false;
        } else {
            if (ImGui::Button("Start Simulation", ImVec2(-1, 30))) is_simulation_running = true;
        }

        ImGui::Spacing();
        ImGui::SliderFloat("Speed", &simulation_speed, 0.5f, 5.0f, "%.1fx");
        
        ImGui::Separator();
        ImGui::Text("Global Status:");
        ImVec4 status_color = is_simulation_running ? ImVec4(0,1,0,1) : ImVec4(1,1,0,1);
        ImGui::TextColored(status_color, "%s", is_simulation_running ? "RUNNING" : "PAUSED");
        ImGui::Text("Active Robots: %d", (int)global_robots.size());
        ImGui::End();

        ImGui::Begin("Robot Status");
        if (ImGui::BeginTable("robot_table", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
            ImGui::TableSetupColumn("ID");
            ImGui::TableSetupColumn("Position");
            ImGui::TableSetupColumn("Battery");
            ImGui::TableSetupColumn("Status");
            ImGui::TableHeadersRow();

            for (const auto& robot : global_robots) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn(); ImGui::Text("R-%d", robot.id);
                ImGui::TableNextColumn(); ImGui::Text("(%.1f, %.1f)", robot.position[0], robot.position[1]);
                
                ImGui::TableNextColumn(); 
                ImVec4 bat_color = ImVec4(1,0,0,1);
                if(robot.battery > 50) bat_color = ImVec4(0,1,0,1);
                else if(robot.battery > 20) bat_color = ImVec4(1,1,0,1);
                ImGui::TextColored(bat_color, "%.1f%%", robot.battery);
                
                ImGui::TableNextColumn(); ImGui::Text("%s", robot.status);
            }
            ImGui::EndTable();
        }
        ImGui::End();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
