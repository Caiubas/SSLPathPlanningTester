import lcm
from data import *
import os


class Globals():
    def __init__(self):
        self.team_blue = False

def handler_tartarus(channel, data_tartarus):

    msg_tartarus = tartarus.decode(data_tartarus)
    #print(f"Received message on channel \"{channel}\"")
    #print(f"   competition mode = {msg_tartarus.competition_mode}")

def handler_GC(channel, data_GC):
    #os.system('clear')
    #print("\n" + "-"*40 + "\n            C E R B E R U S \n" + "-"*40)
    #print(f"Received message on channel \"{channel}\"")
    msg_GC = game_controller.decode(data_GC)
    #print(f"   time_azul    = {msg_GC.team_blue}\n")
    g.team_blue = msg_GC.team_blue
    #print(f"score team blue:   {msg_GC.blue.score} \n")



def handler_vision(channel, data_vision):
    msg_vision = vision.decode(data_vision)

    msg_ia = ia()
    msg_ia.timestamp = msg_vision.timestamp

    if g.team_blue:
        n_robots = msg_vision.robots_blue_size
        robots_source = msg_vision.robots_blue
    else:
        n_robots = msg_vision.robots_yellow_size
        robots_source = msg_vision.robots_yellow

    # robots_size indica quantos robots estão activos
    msg_ia.robots_size = n_robots

    # O array tem sempre 16 slots — preencher os activos, zerar os restantes
    for i in range(16):
        r = msg_ia.robots[i]  # já inicializado com robot() no __init__
        if i < n_robots:
            src = robots_source[i]
            if not src.detected:
                continue
            r.id           = robots_source[i].robot_id
            r.spinner      = False
            r.kick         = False
            r.vel_tang     = 0.0
            r.vel_normal   = 0.0
            r.vel_ang      = 0.0
            r.kick_speed_x = 0.0
            r.kick_speed_z = 0.0
            r.wheel_speed  = False
            r.wheel_fr     = 0.0
            r.wheel_fl     = 0.0
            r.wheel_bl     = 0.0
            r.wheel_br     = 0.0
        # slots i >= n_robots ficam com zeros (já é o default do robot())

    lc.publish("IA", msg_ia.encode())

        

g = Globals()
msg_ia = ia()
lc = lcm.LCM()

subscription = lc.subscribe("tartarus", handler_tartarus)
subscription = lc.subscribe("GC", handler_GC)
subscription = lc.subscribe("vision", handler_vision)

try:
    print("Waiting for messages on channels vision and GC")
    while True:
        lc.handle()
except KeyboardInterrupt:
    print("Shutting down...")
    pass