# node class

class Node:
    def __init__(self,id:int, cost:float, parent:int,heur:float=0.0):
        self.id = id
        self.cost = cost
        self.heur = heur
        self.hcost = self.cost + self.heur
        
        self.parent = parent