#include "handlers.hpp"
#include <algorithm>
#include <iostream>

extern std::mutex data_mutex;
extern LatestData latest_data;

LCMControl lcm_control;

void Handler::handleGC(const lcm::ReceiveBuffer *, const std::string &, const gc_t *msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    msg_GC = *msg;

    if (lcm_control.team_blue_from_lcm) {
        latest_data.team_blue = msg->team_blue;
    }

    latest_data.gc_designated_position_x = msg->designated_position_x;
    latest_data.gc_designated_position_y = msg->designated_position_y;
    latest_data.gc_current_command = msg->current_command;
    latest_data.gc_game_event = msg->game_event;

    // Atualiza blue, exceto goalkeeper_id se desabilitado para LCM
    latest_data.blue.name = msg->blue.name;
    latest_data.blue.score = msg->blue.score;
    latest_data.blue.fouls = msg->blue.fouls;
    if (lcm_control.goalkeeper_id_from_lcm) {
        latest_data.blue.goalkeeper_id = msg->blue.goalkeeper_id;
    }

    // Atualiza yellow, exceto goalkeeper_id se desabilitado para LCM
    latest_data.yellow.name = msg->yellow.name;
    latest_data.yellow.score = msg->yellow.score;
    latest_data.yellow.fouls = msg->yellow.fouls;
    if (lcm_control.goalkeeper_id_from_lcm) {
        latest_data.yellow.goalkeeper_id = msg->yellow.goalkeeper_id;
    }
}


void Handler::handleTartarus(const lcm::ReceiveBuffer *, const std::string &, const tartarus_t *msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    msg_tartarus = *msg;

    std::cout << "[handleTartarus] goal_keeper_id_from_lcm: " << latest_data.goalkeeper_id
            << ", goal_keeper_id recebido do LCM: " << msg->goalkeeper_id << std::endl;

    if (latest_data.goalkeeper_id) {
        if (msg_GC.blue.name == "Cerberus") {
            latest_data.team_blue_status = true;
            latest_data.blue.goalkeeper_id = msg->goalkeeper_id;
        } else if (msg_GC.yellow.name == "Cerberus") {
            latest_data.team_blue_status = false;
            latest_data.yellow.goalkeeper_id = msg->goalkeeper_id;
        } else {
            latest_data.team_blue_status = true;
            latest_data.blue.goalkeeper_id = msg->goalkeeper_id;
        }
    }

    latest_data.ssl_vision = msg->ssl_vision;
    latest_data.autoreferee = msg->autoreferee;
    latest_data.competition_mode = msg->competition_mode;
    latest_data.bool_controller = msg->bool_controller;
    latest_data.debug_mode = msg->debug_mode;
    latest_data.half_field = msg->half_field;
    latest_data.iris_as_GC = msg->iris_as_GC;
    latest_data.right_field = msg->right_field;
    
    latest_data.stm_port = msg->stm_port;
    latest_data.mcast_port_gc = msg->mcast_port_gc;
    latest_data.mcast_port_vision_grsim = msg->mcast_port_vision_grsim;
    latest_data.mcast_port_vision_sslvision = msg->mcast_port_vision_sslvision;
    latest_data.mcast_port_vision_tracked = msg->mcast_port_vision_tracked;
    latest_data.cams_number = msg->cams_number;

    latest_data.designated_position_x = msg->iris_gc.designated_position_x;
    latest_data.designated_position_y = msg->iris_gc.designated_position_y;
    latest_data.current_command = msg->iris_gc.current_command;
    latest_data.game_event = msg->iris_gc.game_event;

    for (const auto& robot : msg->robots) {
        int id    = robot.id;
        int skill = robot.skill;
        int role  = robot.role;

        float movex = robot.move_to_x;
        float movey = robot.move_to_y;
        float turnx = robot.turn_to_x;
        float turny = robot.turn_to_y;

        bool has_kicker = robot.has_kicker;

        latest_data.skill_by_robot[id] = skill;
        latest_data.role_by_robot[id]  = role;
        latest_data.has_kicker[id] = has_kicker;

        // Se usar mapas separados
        latest_data.move_x_by_robot[id] = movex;
        latest_data.move_y_by_robot[id] = movey;
        latest_data.turn_x_by_robot[id] = turnx;
        latest_data.turn_y_by_robot[id] = turny;

        // OU, se usar TargetPose
        // latest_data.target_pose_by_robot[id] = {movex, movey, turnx, turny};

        latest_data.has_kicker[id] = has_kicker;
    }


    latest_data.selected_robot_id = msg->robots->id;
}


void Handler::handleVision(const lcm::ReceiveBuffer *, const std::string &, const vision_t *msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);

    // Atualiza timestamp
    latest_data.timestamp = msg->timestamp;

    // Atualiza bola
    latest_data.balls = msg->balls;

    // Atualiza campo
    latest_data.field = msg->field;

    constexpr int NUM_ROBOTS = 16;

    // ----- Robôs Amarelos -----
    latest_data.robots_yellow.clear();
    latest_data.robots_yellow.reserve(NUM_ROBOTS);

    for (int i = 0; i < NUM_ROBOTS; ++i) {
        data::detection_robots robot{};
        if (i < msg->robots_yellow_size) {
            const auto &src = msg->robots_yellow[i];
            robot.robot_id   = src.robot_id;
            robot.position_x = src.position_x;
            robot.position_y = src.position_y;
            robot.orientation = src.orientation;
            robot.detected   = src.detected;
        } else {
            robot.robot_id   = i;
            robot.position_x = 0.0f;
            robot.position_y = 0.0f;
            robot.orientation = 0.0f;
            robot.detected   = false;
        }
        latest_data.robots_yellow.push_back(robot);
    }
    latest_data.robots_yellow_size = latest_data.robots_yellow.size();

    // ----- Robôs Azuis -----
    latest_data.robots_blue.clear();
    latest_data.robots_blue.reserve(NUM_ROBOTS);

    for (int i = 0; i < NUM_ROBOTS; ++i) {
        data::detection_robots robot{};
        if (i < msg->robots_blue_size) {
            const auto &src = msg->robots_blue[i];
            robot.robot_id   = src.robot_id;
            robot.position_x = src.position_x;
            robot.position_y = src.position_y;
            robot.orientation = src.orientation;
            robot.detected   = src.detected;
        } else {
            robot.robot_id   = i;
            robot.position_x = 0.0f;
            robot.position_y = 0.0f;
            robot.orientation = 0.0f;
            robot.detected   = false;
        }
        latest_data.robots_blue.push_back(robot);
    }
    latest_data.robots_blue_size = latest_data.robots_blue.size();
}




void Handler::handleIA(const lcm::ReceiveBuffer *, const std::string &, const ia_t *msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    msg_ia = *msg;

    latest_data.robots_size = msg->robots_size;
    latest_data.processo = msg->processo;
    latest_data.estrategia = msg->estrategia;
    latest_data.timestamp = msg->timestamp;

    int safe_size = std::max(0, std::min(static_cast<int>(msg->robots_size), 16));

    latest_data.robots.clear();
    for (int i = 0; i < safe_size; ++i) {
        latest_data.robots.push_back(msg->robots[i]);
    }
}

