#pragma once
#include "imgui.h"
#include <vector>
#include <cmath>

// Standalone Robot Waypoint and Pulsing Node Renderer Class
class WaypointRenderer {
public:
    static void Render(
        ImDrawList* draw_list,
        const ImVec2& robot_grid_pos,
        const std::vector<ImVec2>& waypoints,
        const ImVec2& target_grid_pos,
        const ImVec2& canvas_p0,
        float scale_x,
        float scale_y,
        double time,
        ImU32 theme_color = IM_COL32(0, 255, 255, 255)
    ) {
        if (!draw_list) return;

        // 1. Convert grid positions to screen pixel coordinates
        ImVec2 current_pixel = ImVec2(canvas_p0.x + robot_grid_pos.x * scale_x, canvas_p0.y + robot_grid_pos.y * scale_y);
        ImVec2 target_pixel  = ImVec2(canvas_p0.x + target_grid_pos.x * scale_x, canvas_p0.y + target_grid_pos.y * scale_y);

        // 2. Time-based pulse factor (0.0f ~ 1.0f)
        float pulse = (static_cast<float>(std::sin(time * 6.0)) + 1.0f) * 0.5f;

        // 3. Render Waypoint Path and Pulsing Nodes
        if (!waypoints.empty()) {
            ImVec2 last_pixel = current_pixel;
            for (const auto& wp : waypoints) {
                ImVec2 wp_pixel = ImVec2(canvas_p0.x + wp.x * scale_x, canvas_p0.y + wp.y * scale_y);
                
                // Outer glow line & inner crisp neon line
                draw_list->AddLine(last_pixel, wp_pixel, IM_COL32(0, 255, 255, 40), 4.5f);
                draw_list->AddLine(last_pixel, wp_pixel, theme_color, 1.8f);
                
                // Waypoint node center dot & pulsing outer ring
                int alpha = static_cast<int>(150.0f * (1.0f - pulse));
                draw_list->AddCircleFilled(wp_pixel, 2.5f, IM_COL32(255, 255, 255, 220));
                draw_list->AddCircle(wp_pixel, 3.5f + pulse * 4.5f, IM_COL32(0, 255, 255, alpha), 0, 1.5f);

                last_pixel = wp_pixel;
            }
        }

        // 4. Target Marker (Crosshair & Pulsing Ring)
        int target_alpha = static_cast<int>(180.0f * (1.0f - pulse));
        draw_list->AddCircleFilled(target_pixel, 4.0f, IM_COL32(255, 80, 80, 240));
        draw_list->AddCircle(target_pixel, 6.0f + pulse * 6.0f, IM_COL32(255, 80, 80, target_alpha), 0, 2.0f);
        draw_list->AddLine(ImVec2(target_pixel.x - 7.0f, target_pixel.y), ImVec2(target_pixel.x + 7.0f, target_pixel.y), IM_COL32(255, 100, 100, 220), 1.5f);
        draw_list->AddLine(ImVec2(target_pixel.x, target_pixel.y - 7.0f), ImVec2(target_pixel.x, target_pixel.y + 7.0f), IM_COL32(255, 100, 100, 220), 1.5f);

        // 5. Current robot position pulse ring
        int pos_alpha = static_cast<int>(120.0f * (1.0f - pulse));
        draw_list->AddCircle(current_pixel, 10.0f + pulse * 5.0f, IM_COL32(0, 255, 255, pos_alpha), 0, 2.0f);
    }
};
