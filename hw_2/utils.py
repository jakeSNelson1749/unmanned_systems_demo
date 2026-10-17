import math

from hw_2.node import Node

# homework two utilites and tool functions

def calc_Euclidean_dist(point_1:tuple[float, float], 
                        point_2:tuple[float, float]
                        ) -> float:
    # point[0] = North position, point[1] = East position
    return math.sqrt((point_2[0]-point_1[0])**2 + (point_2[1]-point_1[1])**2)
    
# desired angle (in radians) clockwise from North from point 1 to point 2
def desired_course_angle_rad(point_1:tuple[float, float], 
                         point_2:tuple[float, float]
                         ) -> float:
    return math.atan2((point_2[1]-point_1[1]),(point_2[0]-point_1[0]))

# returns -1 if no node found
def get_min_cost(unv: dict[int,Node], astar_flag=False):
    min_id = -1
    min_tracker = float("inf")
    
    for id, node in unv.items():
        
        if(not astar_flag):
            if(node.cost < min_tracker):
                min_tracker = node.cost
                min_id = id
        else:
            if(node.hcost < min_tracker):
                min_tracker = node.hcost
                min_id = id
    return min_id

def polyline_length(waypoints):
    return sum(calc_Euclidean_dist(a,b) for a,b in zip(waypoints[:-1],waypoints[1:]))