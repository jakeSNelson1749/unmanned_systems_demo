import math
import numpy as np

# hw2 imports
import hw_2.utils as utils
from hw_2.node import Node

# grid class
class configuration_space:
    """grid representation for configuration space for pathplanning
    
    instance vars:
        grid_x = (x_min, x_max)
        grid_y = (y_min, y_max)
        resolution (step_size)
        obstacles = [
            obs1 = (North_min, North_max, East_min, East_max)
            obs2...
            ...
            obsN
        ]
    
    """
    
    def __init__(self, grid_east:tuple[int,int], grid_north:tuple[int,int], resolution:int, obstacles: list[tuple[int,...]], safety_rad_m:int):
        self.grid_east = grid_east
        self.grid_north = grid_north
        self.resolution = resolution
        self.obstacles = obstacles
        self.safety_rad = safety_rad_m
        
        self.n_rows:int = (self.grid_north[1] -self.grid_north[0]) // self.resolution
        self.n_cols:int = (self.grid_east[1] -self.grid_east[0]) // self.resolution 
        
        self.inf_obs = self.inflate_obs(self.safety_rad)
        
        # (inflated east min, inflated east max, inflated north min, inflated north max)
        self.inf_bounds = (self.grid_east[0] + self.safety_rad, self.grid_east[1] - self.safety_rad, self.grid_north[0] + self.safety_rad,self.grid_north[1] - self.safety_rad)
        self.ogrid = self.generate_occupancy_grid()

    # use grid bounds and resolution to create occupancy grid
    def generate_occupancy_grid(self):
        ogrid = np.zeros((self.n_rows, self.n_cols), dtype=bool)
        
        # populate
        for row in range(self.n_rows):
            for col in range(self.n_cols):
                center = self.get_point_from_node_id(row*self.n_cols + col)
                ogrid[row,col] = self.collision_check_point(center)
        
        return ogrid
    
    def get_point_from_node_id(self, nodeID:int):
        row, col = divmod(nodeID, self.n_cols)
        
        north = self.grid_north[0] + row*self.resolution + (self.resolution/2)
        east = self.grid_east[0] + col*self.resolution + (self.resolution/2)
        
        return (north,east)

    def get_node_id_from_point(self,point:tuple[float,float]) -> int:
        north = point[0]
        east = point[1]
        
        row_id = math.floor((north-self.grid_north[0])/self.resolution)
        col_id = math.floor((east-self.grid_east[0])/self.resolution)
        
        # clamp (max,max) value to last possible cell
        col_id = min(col_id, self.n_cols-1)
        row_id = min(row_id, self.n_rows-1)
        
        return row_id*self.n_cols + col_id 

    # returns bool representing if point is in an obstacle or out of bounds
    def collision_check_point(self, curr_loc:tuple[float,float]) -> bool:
        # check if within boundaries & not in an (inflated) obs
        curr_north, curr_east = curr_loc[0], curr_loc[1]
        
        if(curr_east < self.inf_bounds[0] or curr_east > self.inf_bounds[1] or curr_north < self.inf_bounds[2] or curr_north > self.inf_bounds[3]):
            return True
        
        # check against obstacles
        for obs in self.inf_obs:
            N_min, N_max, E_min, E_max = obs
            
            if((curr_east >= E_min and curr_east <= E_max) and (curr_north >= N_min and curr_north <= N_max)):
                return True
        
        return False
    
    
    # checks if line-of-sight path between two waypoints contains a collision
    def collision_in_line_of_sight(self, wp1:tuple[float,float], wp2:tuple[float,float], step_size:float = 0.5) -> bool:
        # compute N steps needed
        
        if(wp1 == wp2):
            return self.collision_check_point(wp1)
        
        dist:float = utils.calc_Euclidean_dist(wp1, wp2)
        
        n:int = math.ceil(dist/step_size)
        
        for i in range(n+1):
            t:float = i/n
            stepped_pos_north = wp1[0] + t*(wp2[0] - wp1[0])
            stepped_pos_east = wp1[1] + t*(wp2[1] - wp1[1])
            
            stepped_pos = (stepped_pos_north, stepped_pos_east)
            if(self.collision_check_point(stepped_pos)):
                return True
            
        return False
    
    # mathmatically inflates obstacles 
    def inflate_obs(self, safety_rad) -> list[tuple[int,...]]:
        inflated_obstacles = []
        
        for obs in self.obstacles:
            n_min, n_max, e_min, e_max = obs
            
            temp_inflated_obs = (
                n_min - safety_rad,
                n_max + safety_rad,
                e_min - safety_rad, 
                e_max + safety_rad
            )
            
            inflated_obstacles.append(temp_inflated_obs)
        return inflated_obstacles
    
    def path_ids_to_waypoints(self, path_ids, start, goal):
        waypoints = [start]
        
        start_id = self.get_node_id_from_point(start)
        goal_id = self.get_node_id_from_point(goal)
        
        for id in path_ids:
            if(id == start_id or id==goal_id):
                continue
            waypoints.append(self.get_point_from_node_id(id))
            
        waypoints.append(goal)
        return waypoints
    
    def waypoint_reduction(self, waypoints:list[tuple[float,float]]) ->list[tuple[float,float]]:
        """Reduces waypoint list by line-of-sight checks (with step size 0.5m)"""
        
        start = waypoints[0]
        reduced_waypoints = [start]
        
        curr_wp_index = 0

        while(curr_wp_index < len(waypoints)-1):
            next_wp_index = curr_wp_index +1
            while((next_wp_index +1 < len(waypoints)) and not self.collision_in_line_of_sight(waypoints[curr_wp_index],waypoints[next_wp_index+1])):
                next_wp_index += 1
            reduced_waypoints.append(waypoints[next_wp_index])
            curr_wp_index = next_wp_index
        return reduced_waypoints
    
    def djikstras(self, start:tuple[float,float], goal:tuple[float,float]) -> dict:
        visited = {}
        unvisited = {}
        found = False

        start_cell_id = self.get_node_id_from_point(start)
        goal_cell_id = self.get_node_id_from_point(goal)

        unvisited[start_cell_id] = Node(start_cell_id, 0.0, -1)

        while(len(unvisited) != 0):
            curr_node_id = utils.get_min_cost(unv=unvisited)
            if curr_node_id == -1:
                break
            curr_node = unvisited[curr_node_id]
            curr_north, curr_east = self.get_point_from_node_id(curr_node_id)

            visited[curr_node_id] = curr_node
            del unvisited[curr_node_id]

            # stop once the goal has been expanded
            if curr_node_id == goal_cell_id:
                found = True
                break

            # calculate costs to valid neighbors (double for loop)
            for i in range(-self.resolution, self.resolution + 1, self.resolution):
                for j in range(-self.resolution, self.resolution + 1, self.resolution):
                    if i == 0 and j == 0:   # skip the node itself
                        continue

                    neighbor_north = curr_north + i
                    neighbor_east = curr_east + j

                    # skip if collision (invalid neighbor cell)
                    if(self.collision_check_point((neighbor_north, neighbor_east))):
                        continue

                    # no corner cutting
                    if i != 0 and j != 0:
                        if(self.collision_check_point((neighbor_north, curr_east)) or
                           self.collision_check_point((curr_north, neighbor_east))):
                            continue

                    neighbor_id = self.get_node_id_from_point((neighbor_north, neighbor_east))

                    # already expanded -> its cost is final
                    if neighbor_id in visited:
                        continue

                    # cumulative cost from start
                    edge_cost = utils.calc_Euclidean_dist((curr_north, curr_east), (neighbor_north, neighbor_east))
                    temp_cost = curr_node.cost + edge_cost

                    temp_neighbor = Node(neighbor_id, temp_cost, curr_node_id)

                    # does unvisited already have neighbor?
                    if neighbor_id in unvisited:
                        # compare costs
                        if(temp_cost < unvisited[neighbor_id].cost):
                            # override
                            unvisited[neighbor_id] = temp_neighbor
                    else:
                        unvisited[neighbor_id] = temp_neighbor

        # backtrack from goal cell to return optimal path
        path = []
        if found:
            node_id = goal_cell_id
            while node_id != -1:
                path.append(node_id)
                node_id = visited[node_id].parent
            path.reverse()

        return {
            "success": found,
            "path_ids": path,
            "explored_ids": list(visited.keys()),
            "expanded_nodes": len(visited),
            "num_waypoints": len(path),
            "path_length": visited[goal_cell_id].cost if found else float("inf"),
        }
    
    def a_star(self, start:tuple[float,float], goal:tuple[float,float]) -> dict:
        visited = {}
        unvisited = {}
        found = False

        start_cell_id = self.get_node_id_from_point(start)
        goal_cell_id = self.get_node_id_from_point(goal)

        # heuristic target = center of the goal cell (the node the search actually ends at)
        goal_pt = self.get_point_from_node_id(goal_cell_id)
        start_pt = self.get_point_from_node_id(start_cell_id)
        start_heur = utils.calc_Euclidean_dist(start_pt, goal_pt)

        unvisited[start_cell_id] = Node(start_cell_id, 0.0, -1, start_heur)

        while unvisited:
            curr_node_id = utils.get_min_cost(unv=unvisited, astar_flag=True)
            if curr_node_id == -1:
                break
            curr_node = unvisited[curr_node_id]
            curr_north, curr_east = self.get_point_from_node_id(curr_node_id)

            visited[curr_node_id] = curr_node
            del unvisited[curr_node_id]

            if curr_node_id == goal_cell_id:
                found = True
                break

            for i in range(-self.resolution, self.resolution + 1, self.resolution):
                for j in range(-self.resolution, self.resolution + 1, self.resolution):
                    if i == 0 and j == 0:
                        continue

                    neighbor_north = curr_north + i
                    neighbor_east = curr_east + j

                    if(self.collision_check_point((neighbor_north, neighbor_east))):
                        continue

                    # no corner cutting
                    if i != 0 and j != 0:
                        if(self.collision_check_point((neighbor_north, curr_east)) or
                           self.collision_check_point((curr_north, neighbor_east))):
                            continue

                    neighbor_id = self.get_node_id_from_point((neighbor_north, neighbor_east))

                    if neighbor_id in visited:
                        continue

                    edge_cost = utils.calc_Euclidean_dist((curr_north, curr_east), (neighbor_north, neighbor_east))
                    temp_cost = curr_node.cost + edge_cost
                    neighbor_heur = utils.calc_Euclidean_dist((neighbor_north, neighbor_east), goal_pt)

                    temp_neighbor = Node(neighbor_id, temp_cost, curr_node_id, neighbor_heur)

                    if neighbor_id in unvisited:
                        if temp_cost < unvisited[neighbor_id].cost:   # compare g, not f
                            unvisited[neighbor_id] = temp_neighbor
                    else:
                        unvisited[neighbor_id] = temp_neighbor

        path = []
        if found:
            node_id = goal_cell_id
            while node_id != -1:
                path.append(node_id)
                node_id = visited[node_id].parent
            path.reverse()

        return {
            "success": found,
            "path_ids": path,
            "explored_ids": list(visited.keys()),
            "expanded_nodes": len(visited),
            "num_waypoints": len(path),
            "path_length": visited[goal_cell_id].cost if found else float("inf"),
        }