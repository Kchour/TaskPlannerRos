"""Vehicle + scenario data specified here. 

"""
import pickle
import os

from fastAGC.data_interface import VehicleData, VehicleContainer,\
                                    TravelMapper, RewardMapper

###################################################
# Load pre-generated graphs from environment folder
###################################################
path = os.path.join(os.path.dirname(__file__), "..", "data")
# with open(path+"risk_graph.pkl", 'rb') as handle:
#     risk_graph = pickle.load(handle)
with open(os.path.join(path,"travel_graph_gv.pkl"), 'rb') as h:         
    travel_graph_gv = pickle.load(h)
with open(os.path.join(path,"mission_plans.pkl"), 'rb') as h:         # modify as needed in /data folder  
    mission_plans = pickle.load(h)
with open(os.path.join(path,"rewards.pkl"), 'rb') as h:               # debugging/plotting purposes
    rewards = pickle.load(h)
with open(os.path.join(path,"reward_graph_gv.pkl"), 'rb') as h:       # tHIs will change over time
    reward_graph_gv = pickle.load(h)
with open(os.path.join(path,"reward_graph_av.pkl"), 'rb') as h:       # This will change over time
    reward_graph_av = pickle.load(h)

########################################################
# AV1
########################################################
av1 = VehicleData()
av1.name = "av1"
# config space
av1.config_space = "R2"

# specs
# av1.max_batt_cap = None # joules
av1.top_speed = 16    # m/s
# hard limits constraints
av1.time_lim = 20*60

# mappings
# av1.risk_mapping = None
av1.reward_mapping = RewardMapper(reward_graph_av)

# basic init state information
# av1.pos = (241.354, -205.750)   # northwest corner
# av1.pos = (1046.896, -1113.167)   # southeast corner
# av1.vel = 0.0
# av1.rem_time = 100  # perc, only applies when "max_batt_cap" is set

# user-defined settings, like endurance mode speed
# av1.sel_speed = None

########################################################
# GV1
########################################################

gv1 = VehicleData()
gv1.name = "gv1"

# config space
gv1.config_space = TravelMapper(travel_graph_gv)

# specs
# av1.max_batt_cap = None # joules
gv1.top_speed = 5    # m/s

# hard limits constraints
# actual_time_lim = min(self.time_lim, self.rem_time)
gv1.time_lim = None
# av1.risk_lim = None
# gv1.rendezvous_nodes = [(x,y) for x,y in zip(phase_dict[1]['x'], phase_dict[1]['y'])]

# mappings
# av1.risk_mapping = None
gv1.reward_mapping = RewardMapper(reward_graph_gv)

###########################################
# Add vehicles to list
###########################################

vehicles_container = VehicleContainer()
vehicles_container.add_vehicle(name="av1", type="av", data=av1)
vehicles_container.add_vehicle(name="gv1", type="gv", data=gv1)
