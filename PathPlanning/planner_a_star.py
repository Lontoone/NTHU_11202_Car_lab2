import cv2
import sys
sys.path.append("..")
import PathPlanning.utils as utils
from PathPlanning.planner import Planner
import numpy as np

class AstarNode:
    def __init__(self , x, y , goal , start) :
        self.x = x 
        self.y = y
        #self.g =  start[0] - x + start[1]-y
        #self.h = goal[0] - x + goal[1]-y
        self.g = abs(start[0] - x + start[1]-y)
        self.h = abs(goal[0] - x + goal[1]-y)

        self.h_cost = 1.0
        self.cost = self.g + self.h *self.h_cost

        self.parent = None
        self.processed = False
        
    def set_parent(self , parent , goal , start):
        if self.parent is None:
            self.parent = parent
        else:
            new_g = abs(start[0] - self.x + start[1]-self.y)
            new_h = abs(goal[0] - self.x + goal[1]-self.y)
            new_cost = new_g + new_h * self.h_cost
            if( new_cost < self.cost):
                self.g = new_g 
                self.h = new_h * self.h_cost
                self.cost = new_cost
                self.parent = parent

    pass

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()
    def _search(self , dx , dy, current  , start , goal , img):
        idx_x =  int(current[0] +dx)
        idx_y =  int(current[1] +dy)
        print("search " , (idx_x , idx_y))
        if( idx_y>= img.shape[0] or idx_x >= img.shape[1] or img[idx_y , idx_x , 0]== 0):
            return

        #if (self.nodes[(idx_x , idx_y)]  is None):
        if ((idx_x , idx_y) not in self.nodes ):
            new_node= AstarNode(idx_x , idx_y, goal , start)
            self.nodes[(idx_x , idx_y)]  = new_node
        else:
            new_node = self.nodes[(idx_x , idx_y)]


        self.nodes[(idx_x , idx_y)].set_parent(current , goal , start)
        if(not new_node.processed):
            self.open_nodes[(idx_x , idx_y)] =  self.nodes[(idx_x , idx_y)]
            new_node.processed =True

        

    def create_distance_map (self , size ,center):
        # size:  (y , x )

        #size = (600,525)
        #center = (100,200)

        x = np.abs( np.arange(size[1]) - center[1])
        y = np.abs( np.arange(size[0]) - center[0])
        z = np.array(np.meshgrid(x, y)).T

        return z # [x , y , 2 ]

        return 

    def initialize(self):
        self.queue = []
        self.parent = {}
        self.h = {} # Distance from start to node
        self.g = {} # Distance from node to goal
        self.goal_node = None

        self.step_width = 15 #pixel
        self.nodes = {}

        self.kernels = [
            [-1,1],
            [0,1],
            [1,1],

            [-1,0],
            [1,0],

            [-1,-1],
            [0,-1],
            [1,-1],

        ]


    def planning(self, start=(100,200), goal=(375,520), inter=None, img=None):
        if inter is None:
            inter = self.inter
        start = (int(start[0]), int(start[1]))
        goal = (int(goal[0]), int(goal[1]))
        # Initialize 
        self.initialize()
        self.queue.append(start)
        self.parent[start] = None
        self.g[start] = 0
        self.h[start] = utils.distance(start, goal)
        self.open_nodes ={}

        is_reached = False
        debug_itr = 0
        #while(1):
        while(debug_itr < 5000):
            # TODO: A Star Algorithm           

            # update
            if(len(self.nodes) ==0):
                current_point  = start
            

            for k in self.kernels:
                search_idx =  ( current_point[0] + k[0] * self.step_width ,current_point[1] + k[1] * self.step_width ) 
                self._search(k[0] * self.step_width , k[1]* self.step_width , current_point , start , goal , img )
                
                if (utils.distance( search_idx , goal) < self.step_width):
                    is_reached = True
                    self.nodes[goal] = AstarNode(goal[0] , goal[1], goal , start)
                    self.nodes[goal].set_parent(  ( current_point[0] , current_point[1]) , start,goal  )

            if(is_reached):
                break
            # sort 
            next_point = sorted(self.open_nodes.values(), key=lambda x: x.cost)[0]            
            current_point = (next_point.x , next_point.y)
            del self.open_nodes[current_point]
            #next_point.processed = 999

            #break
            debug_itr +=1

        # Draw points
        ''''''
        print("node length " , len(self.nodes))
        for p in self.nodes:
            print(p)
            img = cv2.circle(img , p , 2 , (255,0,0), -1) 
            img = cv2.putText(img, str(self.nodes[p].cost), p, cv2.FONT_HERSHEY_COMPLEX_SMALL , .55, (0,255,0), 1, cv2.LINE_AA)
            #k = cv2.waitKey(1)
            
        path = []  # output
        path_idx = goal
        path.append(path_idx)
        while(path_idx != start):
            path_idx = self.nodes[path_idx].parent
            path.append(path_idx)
        

        #self.goal_node = (int(200), int(300))
        #self.parent[self.goal_node] = self.goal_node
        # Extract path
        p = self.goal_node
        if p is None:
            return path
        #while(True):
        for i in range(2):

            path.insert(0,p)
            if self.parent[p] is None:
                break
            p = self.parent[p]
        if path[-1] != goal:
            path.append(goal)
        return path
