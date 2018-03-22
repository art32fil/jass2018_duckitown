#!/usr/bin/env python
# sudo apt-get install python-dev graphviz libgraphviz-dev pkg-config
# pip install pygraphviz

import networkx as nx
import sys


def come_is_possible(new_type, new_dir):
    dictionary = {'T-up':    {'left': 1, 'up': 1, 'right': 1, 'down': 0},
                  'T-right': {'left': 0, 'up': 1, 'right': 1, 'down': 1},
                  'T-down':  {'left': 1, 'up': 0, 'right': 1, 'down': 1},
                  'T-left':  {'left': 1, 'up': 1, 'right': 0, 'down': 1},
                  'X':       {'left': 1, 'up': 1, 'right': 1, 'down': 1}}
    return dictionary[new_type][new_dir]


def new_observation(G, new_id, new_type, new_dir, prev_id, prev_dir):
    if (not come_is_possible(new_type, new_dir)):
        print("alarm! we come to " + new_type + " through " + new_dir)
        return
    if (not G.has_node(new_id)):
        G.add_node(new_id, label=str(new_id)+": "+new_type)
    if (len(G) == 1):
        return
    if (G.has_edge(prev_id, new_id) or G.has_edge(new_id, prev_id)):
        return
    G.add_edge(new_id, prev_id, label=new_dir+" -> "+prev_dir)
    G.add_edge(prev_id, new_id, label=prev_dir+" -> "+new_dir)


def is_fully_observed(G, id):
    attributes = nx.get_node_attributes(G, 'label')
    if (attributes[id] == str(id)+": X"):
        return len(G.edges(id)) == 4
    if (attributes[id] == str(id)+": T-up" or
            attributes[id] == str(id)+": T-right" or
            attributes[id] == str(id)+": T-down" or
            attributes[id] == str(id)+": T-left"):
        return len(G.edges(id)) == 3
    return -1


def possible_dirs(G, id):
    attributes = nx.get_node_attributes(G, 'label')
    if (attributes[id] == str(id)+": T-up"):
        return ['left', 'up', 'right']
    if (attributes[id] == str(id)+": T-right"):
        return['down', 'up', 'right']
    if (attributes[id] == str(id)+": T-down"):
        return['left', 'down', 'right']
    if (attributes[id] == str(id)+": T-left"):
        return['left', 'up', 'down']
    if (attributes[id] == str(id)+": X"):
        return['left', 'up', 'down', 'right']

def get_id_dir_for_move(G, prev_id, prev_dir):
    # all links are set
    # output - list of 3-element set
    #   where we could go (dir),
    #     where we will come (id),
    #       in what dir we will come (dir)
    Es = G.edges(prev_id, data=True)
    observed_dirs = []
    observed_nodes = []
    incomming_dirs = []
    out = []
    for edge in Es:
        if (prev_dir != edge[2]['label'].split(' -> ')[0]):
            out.append([edge[2]['label'].split(' -> ')[0],
                        edge[1],
                        edge[2]['label'].split(' -> ')[1]])
    return out

def found_path_to_unobserved(G, id, prev_dir, list_of_observed):
    #print ("we were here: "+str(list_of_observed))
    npm = new_possible_moves(G, id, prev_dir)
    if (len(npm) != 0):
       #print ("I ("+str(id)+") have unobserved "+str(npm))
       return True, [npm[0]]
    #print ("I ("+str(id)+") have not unobserved, go to child")
    moving_array = get_id_dir_for_move(G, id, prev_dir)
    #print ("found children: " + str(moving_array))
    for child in moving_array:
        #print ("go to child " + str(child))
        if ([child[1], child[0]] in list_of_observed):
            continue
        #print ("this child was not visited yet")
        b, path = found_path_to_unobserved(G, child[1], child[2],
                                           list_of_observed+[[child[1], child[0]]])
        if (b):
            #print ("my ("+str(id)+") child ("+str(child[1])+") has unobserved way, so i do "+str(child[0]))
            return True, [child[0]]+path
    #print ("I ("+str(id)+") have not unobserved "+str(npm))
    return False, []

def i_has_unobserved_way(G, id, prev_dir):
    return found_path_to_unobserved(G, id, prev_dir, [])

def observed_dirs(G, id):
    Es = G.edges(id, data=True)
    observed_dirs = []
    for edge in Es:
        observed_dirs.append(edge[2]['label'].split(' -> ')[0])
    return observed_dirs


def unobserved_dirs(G, id):
    if (is_fully_observed(G, id)):
        return []
    return list(set(possible_dirs(G, id)) - set(observed_dirs(G, id)))


def all_possible_moves(G, id, prev_dir):
    return list(set(possible_dirs(G, id)) - set([prev_dir]))


def new_possible_moves(G, id, prev_dir):
    return list(set(unobserved_dirs(G, id)) - set([prev_dir]))
    

G = nx.MultiDiGraph()

diery = ['0', 'X', 'X', 'X', 'T-right', 'X', 'T-up', 'T-left', 'T-up']

def step(incomming_id, incomming_type, incomming_dir, outcomming_id, outcomming_dir):
    print("found: "+str(incomming_id)+", come from "+incomming_dir)
    new_observation(G,incomming_id,incomming_type,incomming_dir,outcomming_id,outcomming_dir)
    b, dir = i_has_unobserved_way(G,incomming_id,incomming_dir)
    if (b):
        print("path to unobserved: "+str(dir))
    return b, dir

carry_on = True
outcommimg_id = 0
outcomming_dir = '0'
while (carry_on):
    sys.stdout.write("found id: ")
    id = input()
    print("has type: "+diery[id])
    sys.stdout.write("incomming dir: ")
    incomming_dir = raw_input()
    carry_on, path = step(id,diery[id],incomming_dir,outcommimg_id,outcomming_dir)
    if (not carry_on)
        continue
    for moving in path:
        Es = G.edges(id, data=True)
        for edge in Es:
            if (moving == edge[2]['label'].split(' -> ')[0]):
                id = edge[1]
                incomming_dir = edge[2]['label'].split(' -> ')[1]
                print("go "+moving+" to " + str(id) + " from " + incomming_dir)
    
    outcommimg_id = id
    outcomming_dir = path[-1]
    nx.nx_agraph.write_dot(G, "/home/user/git/jass2018_duckitown/graph/graph.txt")

