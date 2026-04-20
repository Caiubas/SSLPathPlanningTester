#include "include/vision_master.hpp"
#include "include/tartarus.hpp"

data::autoref my_autoref_data;

void vision_master::recebe_dados_tracked() {
    TrackerWrapperPacket tracked_packet;
    TrackedFrame tracked_frame;

    data::detection_robots_tracked new_robot;

    struct sockaddr_in sender_addr;
    socklen_t addr_len = sizeof(sender_addr);

    for(int i = 0; i < vision_master_instance.cameras ; i++){ // usar 4 para grsim e usar 1 para ssl-vision
        int bytes_received_TRACKED = recvfrom(sock_TRACKED, buffer_TRACKED, BUFFER_SIZE, 0, (struct sockaddr*)&sender_addr, &addr_len);
        //std::cout << "bytes_received_TRACKED: " << bytes_received_TRACKED << std::endl;

        if (bytes_received_TRACKED > 0) {
            // Parse dos dados recebidos (Tracked)
            tracked_packet.ParseFromArray(buffer_TRACKED, bytes_received_TRACKED);    
            if(tracked_packet.has_tracked_frame()){
                tracked_frame = tracked_packet.tracked_frame();
                my_autoref_data.timestamp = tracked_frame.frame_number();

                if (tracked_frame.robots_size() > 0) {
                    for (int i = 0; i < tracked_frame.robots_size(); i++) {
                        RobotId new_robot_id = tracked_frame.robots(i).robot_id();
                        if(new_robot_id.team() == 2){ //UNKNOWN = 0, YELLOW = 1, BLUE = 2
                            if(!han.new_tartarus.half_field || (han.new_tartarus.half_field && ((han.new_tartarus.right_field && tracked_frame.robots(i).pos().x() > 0) || (!han.new_tartarus.right_field && tracked_frame.robots(i).pos().x() < 0)))) {
                                my_vision_data.robots_blue[new_robot_id.id()].detected = true;
                                my_vision_data.robots_blue[new_robot_id.id()].robot_id = new_robot_id.id();
                                my_vision_data.robots_blue[new_robot_id.id()].position_x = tracked_frame.robots(i).pos().x()*1000; // mm
                                my_vision_data.robots_blue[new_robot_id.id()].position_y = tracked_frame.robots(i).pos().y()*1000;
                                my_autoref_data.robots_blue[new_robot_id.id()].vel_x = tracked_frame.robots(i).vel().x()*1000; // mm/s
                                my_autoref_data.robots_blue[new_robot_id.id()].vel_y = tracked_frame.robots(i).vel().y()*1000;
                                my_autoref_data.robots_blue[new_robot_id.id()].vel_angular = tracked_frame.robots(i).vel_angular(); //rad/s
                                my_vision_data.robots_blue[new_robot_id.id()].orientation = tracked_frame.robots(i).orientation();
                                vision_master_instance.blue_ids.insert(new_robot_id.id());
                            }
                        }
                        
                    
                        else if(new_robot_id.team() == 1){ //UNKNOWN = 0, YELLOW = 1, BLUE = 2
                        // Para os robôs amarelos, repita o mesmo processo
                            if(!han.new_tartarus.half_field || (han.new_tartarus.half_field && ((han.new_tartarus.right_field && tracked_frame.robots(i).pos().x() > 0) || (!han.new_tartarus.right_field && tracked_frame.robots(i).pos().x() < 0)))) {
                                my_vision_data.robots_yellow[new_robot_id.id()].detected = true;
                                my_vision_data.robots_yellow[new_robot_id.id()].robot_id = new_robot_id.id();
                                my_vision_data.robots_yellow[new_robot_id.id()].position_x = tracked_frame.robots(i).pos().x()*1000; // mm
                                my_vision_data.robots_yellow[new_robot_id.id()].position_y = tracked_frame.robots(i).pos().y()*1000; // mm
                                my_autoref_data.robots_yellow[new_robot_id.id()].vel_x = tracked_frame.robots(i).vel().x()*1000; // mm/s
                                my_autoref_data.robots_yellow[new_robot_id.id()].vel_y = tracked_frame.robots(i).vel().y()*1000; // mm/s
                                my_autoref_data.robots_yellow[new_robot_id.id()].vel_angular = tracked_frame.robots(i).vel_angular(); //rad/s
                                my_vision_data.robots_yellow[new_robot_id.id()].orientation = tracked_frame.robots(i).orientation();
                            }
                                vision_master_instance.yellow_ids.insert(new_robot_id.id());
                        }
                        else{
                            std::cout << "!!!! há um robô com ID de time desconhecido !!!!" << std::endl;
                        }
                    }
                }
            
                if (tracked_frame.balls_size() > 0) {
                    int balls_detected = 0;
                    int max_confidence_idx = -1;
                    float max_confidence = 0;
                    for(int i = 0; i < tracked_frame.balls_size(); i++) {
                        Vector3 ball_pos = tracked_frame.balls(i).pos();
                         // Se half_field está ativado, só associa a bola do lado correto do campo
                        if(!han.new_tartarus.half_field || (han.new_tartarus.half_field && ((han.new_tartarus.right_field && ball_pos.x() > 0) || (!han.new_tartarus.right_field && ball_pos.x() < 0)))) {
                            if (tracked_frame.balls(i).visibility() > max_confidence) {
                                max_confidence = tracked_frame.balls(i).visibility();
                                max_confidence_idx = i;
                            }
                        }
                    }
                    if(han.new_tartarus.autoreferee == true && max_confidence_idx != -1) {
                        Vector3 ball_pos = tracked_frame.balls(max_confidence_idx).pos();
                        my_vision_data.balls.position_x = ball_pos.x()*1000;
                        my_vision_data.balls.position_y = ball_pos.y()*1000;
                        if(tracked_frame.balls(max_confidence_idx).has_vel()) {
                            Vector3 ball_vel = tracked_frame.balls(max_confidence_idx).vel();
                            my_autoref_data.balls.vel_x = ball_vel.x()*1000;//multiplicando por 1000 para converter de metros para milimetros
                            my_autoref_data.balls.vel_y = ball_vel.y()*1000;
                        }
                    }
                }
            }
        }
    }
    my_vision_data.robots_yellow_size = vision_master_instance.yellow_ids.size();
    my_vision_data.robots_blue_size = vision_master_instance.blue_ids.size();
    //std::cout << "Timestamp: " << my_autoref_data.timestamp << std::endl;
}


