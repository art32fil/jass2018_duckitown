#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTagsWithInfos, BoolStamped
from std_msgs.msg import String, Int16 #Imports msg
import networkx as nx
import time

def extract_info(dec_digit):
    digit = bin(dec_digit)
    digit_len = len(digit)-2
    cnt_zero = 16 - digit_len
    zeros = ''
    for i in range(cnt_zero):
        zeros += '0'
    t = digit.split('0b')
    digit = '0b' + zeros + t[1]
    print(digit)
    array = []
    two_last = digit[-2:]
    if int(two_last, 2) == 0: array.append("up")
    elif int(two_last, 2) == 1: array.append("right")
    elif int(two_last, 2) == 2: array.append("down")
    else: array.append("left")
    cross = digit[-5:-2]
    if int(cross, 2) == 0: array.append("X")
    elif int(cross, 2) == 1: array.append("T-down")
    elif int(cross, 2) == 2: array.append("T-left")
    elif int(cross, 2) == 3: array.append("T-up")
    else: array.append("T-right")
    num = digit[:-5]
    array.append(int(num, 2))
    array.reverse()
    return array

def come_is_possible(new_type, new_dir):
    dictionary = {'T-up':    {'left': 1, 'up': 1, 'right': 1, 'down': 0},
                  'T-right': {'left': 0, 'up': 1, 'right': 1, 'down': 1},
                  'T-down':  {'left': 1, 'up': 0, 'right': 1, 'down': 1},
                  'T-left':  {'left': 1, 'up': 1, 'right': 0, 'down': 1},
                  'X':       {'left': 1, 'up': 1, 'right': 1, 'down': 1},
                  '|':       {'left': 0, 'up': 1, 'right': 0, 'down': 1}}
    return dictionary[new_type][new_dir]


def new_observation(G, new_id, new_type, new_dir, prev_id, prev_dir):
    rospy.loginfo("try to add edge: "+str(new_id)+ ": " + new_dir+" -> "+prev_dir + ": " + str(prev_id))
    rospy.loginfo("try to add edge: "+str(prev_id)+ ": " + prev_dir+" -> "+new_dir + ": " + str(new_id))
    if (not come_is_possible(new_type, new_dir)):
        rospy.loginfo("alarm! we come to " + new_type + " through " + new_dir)
        return
    rospy.loginfo("we come from right lodirection")
    if (not G.has_node(new_id)):
        G.add_node(new_id, label=str(new_id)+": "+new_type)
    rospy.loginfo("new id " + str(id) + "is in graph (added or already was)")
    if (prev_id == -2):
        rospy.loginfo ("prev id is -2, so we don't add edge")
        return
    rospy.loginfo ("prev id is not -2. carry on")
    if (G.has_edge(prev_id, new_id) or G.has_edge(new_id, prev_id)):
        rospy.loginfo("this edge already exists")
        return
    rospy.loginfo("edges are added")
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
    if (attributes[id] == str(id)+": |"):
        return len(G.edges(id)) == 2
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
    if (attributes[id] == str(id)+": |"):
        return['up', 'down']

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

def found_path_to_unobserved(G, id, prev_dir, list_of_observed, possible_moves):
    #print ("we were here: "+str(list_of_observed))
    npm = possible_moves(G, id, prev_dir)
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
                                           list_of_observed+[[child[1], child[0]]],
                                           possible_moves)
        if (b):
            #print ("my ("+str(id)+") child ("+str(child[1])+") has unobserved way, so i do "+str(child[0]))
            return True, [child[0]]+path
    #print ("I ("+str(id)+") have not unobserved "+str(npm))
    return False, []

def i_has_unobserved_way(G, id, prev_dir):
    return found_path_to_unobserved(G, id, prev_dir, [], new_possible_moves)

def find_path_to_airport(G, id, prev_dir):
    if (id == -1):
       return []
    

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

def step(G, incomming_id, incomming_type, incomming_dir, outcomming_id, outcomming_dir):
    rospy.loginfo("found: "+str(incomming_id)+", come from "+incomming_dir)
    new_observation(G,incomming_id,incomming_type,incomming_dir,outcomming_id,outcomming_dir)
    b, dir = i_has_unobserved_way(G,incomming_id,incomming_dir)
    if (b):
        rospy.loginfo("path to unobserved: "+str(dir))
    return b, dir

def own_dir(incomming_dir, outcomming_id): # 0 - turn left, 1 - forward, 2 - right
    if   (incomming_dir == 'up'):
        if   (outcomming_id == 'up'):
            rospy.logionf("alarm! we need to go up from up! bot will come in the random direction")
            return [0,1,2]
        elif (outcomming_id == 'right'):
            return [0]
        elif (outcomming_id == 'down'):
            return [1]
        elif (outcomming_id == 'left'):
            return [2]
    elif (incomming_dir == 'right'):
        if   (outcomming_id == 'up'):
            return [2]
        elif (outcomming_id == 'right'):
            rospy.logionf("alarm! we need to go right from right! bot will come in the random direction")
            return [0,1,2]
        elif (outcomming_id == 'down'):
            return [0]
        elif (outcomming_id == 'left'):
            return [1]
    elif (incomming_dir == 'down'):
        if   (outcomming_id == 'up'):
            return [1]
        elif (outcomming_id == 'right'):
            return [2]
        elif (outcomming_id == 'down'):
            rospy.logionf("alarm! we need to go down from down! bot will come in the random direction")
            return [0,1,2]
        elif (outcomming_id == 'left'):
            return [0]
    elif (incomming_dir == 'left'):
        if   (outcomming_id == 'up'):
            return [0]
        elif (outcomming_id == 'right'):
            return [1]
        elif (outcomming_id == 'down'):
            return [2]
        elif (outcomming_id == 'left'):
            rospy.logionf("alarm! we need to go left from left! bot will come in the random direction")
            return [0,1,2]


class RandomAprilTagTurnsNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)


        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz
        self.G = nx.MultiDiGraph()
        self.outcomming_id = -2
        self.outcomming_dir = '0'
        self.path = []
        self.map_observing = True
        self.prev_invoke_time = time.time()
        self.airport_x = 0
        self.airport_y = 0

    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode != mode_msg.INTERSECTION_CONTROL):
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def cbTag(self, tag_msgs):
        if(self.fsm_mode == "INTERSECTION_CONTROL"):
            self.pub_turn_type.publish(self.turn_type)
            for detection in tag_msgs.detections:
                curr_invoke_time = time.time()
                if (curr_invoke_time - self.prev_invoke_time < 10):
                    self.prev_invoke_time = curr_invoke_time
                    self.pub_turn_type.publish(self.turn_type)
                    return
                self.prev_invoke_time = curr_invoke_time
                if (detection.id == -1):
                    rospy.loginfo("aiport found on" + 
                                  detection.pose.header.frame_id.split(" ")[-2] + 
                                  detection.pose.header.frame_id.split(" ")[-1])
                    id = -1
                    type = "|"
                    incomming_dir = 'down'
                    self.airport_x = detection.pose.header.frame_id.split(" ")[-2]
                    self.airport_y = detection.pose.header.frame_id.split(" ")[-1]
                else:
                    id, type, incomming_dir = extract_info(detection.id)
                rospy.loginfo("found id: " + str(id))
                rospy.loginfo("has type: " + type)
                rospy.loginfo("incomming dir: " + incomming_dir)
                rospy.loginfo("previous id: " + str(self.outcomming_id))
                rospy.loginfo("outcomming dir: " + str(self.outcommimg_dir))
                if (len(self.path) == 0):
                    rospy.loginfo("self.path = " + str(self.path))
                    rospy.loginfo("self.outcomming_dir = " + str(self.outcomming_dir))
                    self.map_observing, self.path = step(self.G,
                                                         id,
                                                         type,
                                                         incomming_dir,
                                                         self.outcomming_id,
                                                         self.outcomming_dir)
                    nx.nx_agraph.write_dot(self.G, "/home/ubuntu/graph.txt")
                    if (self.map_observing == False):
                        self.path = found_path_to_unobserved(self.G,
                                                             id,
                                                             incomming_dir,
                                                             [],
                                                             find_path_to_airport)

                availableTurns = own_dir(incomming_dir, self.path[0])
                self.outcomming_dir = self.path[0]
                self.outcomming_id = id
                self.path = self.path[1:]

                if(len(availableTurns)>0):
                    randomIndex = numpy.random.randint(len(availableTurns))
                    chosenTurn = availableTurns[randomIndex]
                    self.turn_type = chosenTurn
                    self.pub_turn_type.publish(self.turn_type)
                    rospy.loginfo("possible turns %s." %(availableTurns))
                    rospy.loginfo("Turn type now: %i" %(self.turn_type))


    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('random_april_tag_turns_node', anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode()

    # Setup proper shutdown behavior
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
