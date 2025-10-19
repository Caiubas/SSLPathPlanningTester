#ifndef VISION_MASTER_HPP
#define VISION_MASTER_HPP

#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unordered_set>
#include <thread>
#include "socket_connect.hpp"
#include "../../data_lcm/vision.hpp"
#include "../../build/ssl_vision_wrapper.pb.h"
#include "../../data_lcm/autoref.hpp"
#include "../../build/ssl_vision_wrapper_tracked.pb.h"
#include <chrono>
#include <unordered_map>

using namespace std::chrono;

class vision_master
{
public:
    void vision_main();
    void recebe_dados_tracked();
    void recebe_dados_vision();

    lcm::LCM lcm;
    bool ssl_vision_atual;
    bool autoreferee_atual;
    int cameras;
    std::unordered_set<int> yellow_ids;
    std::unordered_set<int> blue_ids; // Conjunto para armazenar IDs únicos, de robôs

    steady_clock::time_point last_detected_ball;
    bool ball_detected_once = false;
    bool ball_detected = false;
};

extern vision_master vision_master_instance;
extern data::vision my_vision_data;
extern data::autoref my_autoref_data;

#endif