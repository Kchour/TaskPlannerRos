"""Creates graph (and other data) pertaining to phase scene:
    - travel_graph_av.pkl
    - reward_graph_gv.pkl
    - reward_graph_av.pkl
    - rewards.pkl
    - mission_plans.pkl 

"""
from copy import deepcopy
from typing import List, Union
import pandas as pd
import numpy as np
import pickle
from fastAGC.misc import MissionPlan

from fastAGC.graph.graph_manager import GraphFactory

# Mission Start, Mission End
mission_start = (740978.364, 3390521.932)
# mission_end = (740978.364, 3390521.932)
num_of_mission_phases = 3

###########################################################
# mission planner (simulation purposes) format given below
###########################################################
#
# mission_plan_container:
#     phase_id_1 (int):
#         team_id_1 (int): 
#               vehicle_name_1 (str): MissionPlan object (see import)
#               vehicle_name_2 (str): MissionPlan object
#         team_id_2 (int): MissionPlan object
#         .
#         .
#         .
#     phase_id_N (int)
#     .
#     .
#     .

# [phase_id][team_id][vehicle_name]
mission_plan_container = {}

# init mission plan container for team1 and team 2
# for 3 phases
for i in range(num_of_mission_phases):
    phase_id = i+1
    mission_plan_container[phase_id] = {1: 
                {'gv1': MissionPlan(team_id=1, vehicle_name='gv1', phase_id=phase_id),
                 'av1': MissionPlan(team_id=1, vehicle_name='av1', phase_id=phase_id)}
                                        }

# set the start of the mission and end location per team
mission_plan_container[1][1]['gv1'].start = mission_start
mission_plan_container[1][1]['av1'].start = mission_start
# mission_plan_container[3][1].end = #(Optional)

# mission_plan_container[1][2].start = (740978.364, 3390521.932)
# mission_plan_container[3][2].end = #(Optional)

########################
# load all the files
########################
phase_data = pd.read_csv("phase_line_v4.csv", header=None, skiprows=[0])        # phase line (not on graph) (missing headers)
cen_loc = pd.read_csv("centroid_locations_v4.csv")                              # all node locations
dist_cen = pd.read_csv("dist_centroid_v4.csv", delimiter=',')                   # edge costs
tri_types = pd.read_csv("triangle_type_v4.csv", delimiter=',')                  # noi/checkpoint and phase it belongs to
phase_nodes = pd.read_csv("phase_lineID_v4.csv", header=None, skiprows=[0])                 # actual phase node id (missing headers)

#########################################################
# Create a dict for phase_lines (ACTUAL UTM, DEBUGGING PURPOSES)
# {id (int) : 
#   {'x' (str): [...(float)...]
#    'y' (str): [...(float)...]
#   } 
# }
# MAY HAVE TO OPEN AND RESAVE THIS CSV FILE
#########################################################

phase_dict = {}
# remove any whitespaces from headings
phase_data = phase_data.applymap(lambda x: x.strip() if type(x)==str else x)
for index, row in phase_data.iterrows(): 
    id = int(row[0])
    # init
    if id not in phase_dict:
        phase_dict[id] = {}
    # get coordinates
    phase_dict[id]["x"] = np.round(row[1::2].values, 3)
    phase_dict[id]["y"] = np.round(row[2::2].values, 3)

#########################################################
# create lookup of all centroid locations (traversibility graph)
# {nodeID (int): (x_coord (float), y_coord (float)) }
#########################################################

# remove any whitespaces from headings
cen_loc.rename(columns=lambda x: x.strip(), inplace=True)
# process dataframe
cen_loc_table = {}
for index, row in cen_loc.iterrows():
    nodeID = int(row["nodeID"])
    x_coord = row["X_centroid"]
    y_coord = row["Y_centroid"]
    cen_loc_table[nodeID] = (round(x_coord, 3), round(y_coord, 3))

#########################################################
# process dist_centroid for adj list
# {(v1 (float), v2 (float)): dist (float)}, where
# v1 = cen_loc_table[nodeID1]
# v2 = cen_loc_table[nodeID2]
#########################################################

# remove any whitespaces from headings
dist_cen.rename(columns=lambda x: x.strip(), inplace=True)
# build edge dict of the travel graph
tv_edge_dict = {}
for index, row in dist_cen.iterrows():

    # get source vertex
    nodeID = int(row["nodeID"])

    # get neighbors, distances, and attempt to store it
    try:
        v = int(row["neighbourID_1"])
        dist = row["distance_1"]
        tv_edge_dict[(cen_loc_table[nodeID], cen_loc_table[v])] = dist
    except:
        pass

    try:
        v= int(row["neighbourID_2"])
        dist = row["distance_2"]
        tv_edge_dict[(cen_loc_table[nodeID], cen_loc_table[v])] = dist
    except:
        pass

    try:
        v = int(row["neighbourID_3"])
        dist = row["distance_3"]
        tv_edge_dict[(cen_loc_table[nodeID], cen_loc_table[v])] = dist
    except:
        pass
#########################################################
# Figure out "rewards" given by :
# - checkpoints:  must be visited
# - NAI: are visited by AVs
# rewards = {v (float): reward (float, int) }, where
# v = cen_loc_table[nodeID]
# TODO: NEED TO FIGURE WHICH PHASELINES THE REWARDS LIE IN
#########################################################

rewards = {"checkpoint": set(),
           "NAI": set()
}

# remove any whitespaces from headings
tri_types.rename(columns=lambda x: x.strip(), inplace=True)
# remove white spaces from rest of dataframe
tri_types = tri_types.applymap(lambda x: x.strip() if type(x)==str else x)
for index, row in tri_types.iterrows(): 
    ########################################
    # TUNE THE REWARDS HERE
    ########################################
    pos = cen_loc_table[int(row["triangle_ID"])]
    type = row["triangle_type"]
    rewards[type].add(pos)
    phase_id = row["PhaseID"]

    # also update mission plan for each team
    # assume each team will have the same checkpoint and nai
    if "checkpoint" == type:
        # mission_plan_container[phase_id][1].checkpoint.append(pos)
        for team in mission_plan_container[phase_id]:
            mission_plan_container[phase_id][team]['gv1'].checkpoint.append(pos)
            mission_plan_container[phase_id][team]['gv1'].reward_values.update({pos: 5})

            mission_plan_container[phase_id][team]['av1'].checkpoint.append(pos)
            mission_plan_container[phase_id][team]['av1'].reward_values.update({pos: 5})

    elif "NAI" == type:
        # mission_plan_container[phase_id][1].nai.append(pos)
        for team in mission_plan_container[phase_id]:
            mission_plan_container[phase_id][team]['av1'].nai.append(pos)
            mission_plan_container[phase_id][team]['av1'].reward_values.update({pos: 10})


##################################################
#   Process travel graph so that it is connected!
##################################################
from fastAGC.search.search import BestFirstSearch
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from fastAGC.animation import AnimateV2

#######################################
# plot the original graph 
#######################################
fig, ax = plt.subplots(figsize=(7,7))
travel_graph_gv = GraphFactory.create_graph("Generic", edge_dict=tv_edge_dict, vertex_dict=None, graph_type="undirected", deep_copy=False)

# Equal axes aspect ratio
ax.set_aspect("equal")

# create LineCollection artist object to see all edges
line_segments = LineCollection(list(travel_graph_gv.edges), colors="gray", alpha=0.25)
# This returns the artist, same object as "line_segments"
ax.add_collection(line_segments)

####################################################################################
# find one connected graph
####################################################################################
# bfs = BestFirstSearch(travel_graph_gv, (740978.364, 3390521.932)) #top-left corner
bfs = BestFirstSearch(travel_graph_gv, mission_start) #top-left corner
bfs.use_algorithm()

# find symmetric difference
to_remove = set(travel_graph_gv.vertices)^set(bfs.g)

# remove non-connected nodes from graph
travel_graph_gv.remove_vertices(to_remove)

# create LineCollection artist object to see all edges
line_segments = LineCollection(list(travel_graph_gv.edges), colors="orange", zorder=1)
# This returns the artist, same object as "line_segments"
ax.add_collection(line_segments)

#####################################
# Plot checkpoints and OOIs
#####################################
data = np.array(list(rewards["checkpoint"]))    
ax.scatter(data[:,0], data[:,1], s=150, marker='o', alpha=1, zorder=2) 

data = np.array(list(rewards["NAI"]))
ax.scatter(data[:,0], data[:,1], s=150, marker='*', alpha=1, zorder=2)

# Plot actual phase lines (given by utm coordinates)
ax.plot(phase_dict[1]["x"], phase_dict[1]["y"], 'b')
ax.plot(phase_dict[2]["x"], phase_dict[2]["y"], 'r')

###########################################################
# make sure rendezvous is a subset of the "connected" travel_graph_gv
# Find the nearest points
############################################################
valid_rendezvous_locs_dict = {}
for key in phase_dict.keys():
    rendezvous_locs = [(x,y) for x,y in zip(phase_dict[key]['x'], phase_dict[key]['y'])]
    valid_rendezvous_locs = []
    for pt in rendezvous_locs:
        gv_tv_space = np.array(list(travel_graph_gv.vertices))
        diff = np.array(pt) - gv_tv_space
        dist = np.hypot(*diff.T)
        min_dist = min(dist)
        valid_idx = np.where(dist <= min_dist)
        valid_gv_pt = list(map(tuple, gv_tv_space[valid_idx].tolist()))

        valid_rendezvous_locs.extend(valid_gv_pt)
    
    # make sure valid points are a subset of travel_graph_gv
    valid_rendezvous_locs_dict[key] = list(set(valid_rendezvous_locs).intersection(set(travel_graph_gv.vertices)))

# Add valid_rendezvous_locs to mission_plan_container
mission_plan_container[1][1]['gv1'].end = list(valid_rendezvous_locs_dict[1])
mission_plan_container[2][1]['gv1'].end = list(valid_rendezvous_locs_dict[2])
mission_plan_container[1][1]['av1'].end = list(valid_rendezvous_locs_dict[1])
mission_plan_container[2][1]['av1'].end = list(valid_rendezvous_locs_dict[2])


# plot valid_rendezvous_locs
vrld1 = np.array(list(valid_rendezvous_locs_dict[1]))
vrld2 = np.array(list(valid_rendezvous_locs_dict[2]))

sct = ax.scatter(vrld1[:,0], vrld1[:,1], s=100, alpha=1, marker='s')
# AnimateV2.add_artist_ex(sct, 'vrld1')

sct = ax.scatter(vrld2[:,0], vrld2[:,1], s= 100, alpha=1, marker='s')
# AnimateV2.add_artist_ex(sct, 'vrld2')
# AnimateV2.update()
# init
# AnimateV2.init_figure(fig, ax)

# call to autoscale axes limits
ax.autoscale()

###############################
# TUNE BASE REWARDS HERE. Assume it is 0
###############################
# gv can only visit checkpoints

reward_vertex_dict_gv = {x: 0 for x in rewards["checkpoint"]}

# av can visit either one
reward_vertex_dict_av = {x: 0 for x in rewards["checkpoint"]}
reward_vertex_dict_av.update({x: 0 for x in rewards["NAI"]})
#############################################
# Build graphs and save
#############################################
travel_graph_gv = GraphFactory.create_graph("Generic", edge_dict=tv_edge_dict, vertex_dict=None, graph_type="undirected")
reward_graph_gv = GraphFactory.create_graph("Generic", edge_dict=None, vertex_dict=reward_vertex_dict_gv)
reward_graph_av = GraphFactory.create_graph("Generic", edge_dict=None, vertex_dict=reward_vertex_dict_av)

plt.show()

# SAVE graphs/lookup tables as pickle files
with open("travel_graph_gv.pkl", "wb") as h:
    pickle.dump(travel_graph_gv, h)

with open("mission_plans.pkl", "wb") as h:
    pickle.dump(mission_plan_container, h) 

with open("rewards.pkl", "wb") as h:
    pickle.dump(rewards, h) 

with open("reward_graph_gv.pkl", "wb") as h:
    pickle.dump(reward_graph_gv, h)

with open("reward_graph_av.pkl", "wb") as h:
    pickle.dump(reward_graph_av, h)

print("Wrote pkl files!")
