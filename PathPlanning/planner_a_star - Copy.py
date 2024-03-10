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
        self.g = start[0] - x + start[1]-y
        self.h = goal[0] - x + goal[1]-y
        self.parent = None
    def set_parent(self , parent , goal , start):
        if self.parent is None:
            self.parent = parent
        else:
            new_g = start[0] - self.x + start[1]-self.y
            new_h = goal[0] - self.x + goal[1]-self.y

            if(new_g + new_h < self.g + self.h):
                self.g = new_g 
                self.h = new_h
                self.parent = parent

    pass

class PlannerAStar(Planner):
    def __init__(self, m, inter=10):
        super().__init__(m)
        self.inter = inter
        self.initialize()
    def _search(self , idx, current  , start , goal , img):
        #idx_x = current[0] +dx
        #idx_y = current[1] +dy
        if(img[idx , idx])==0:
            return

        if (self.nodes[(idx[0] , idx[1])]  is None):
            self.nodes[(idx[0] , idx[1])]  = AstarNode(idx[0] , idx[1], goal , start)
        self.nodes[(idx[0] , idx[1])].set_parent(current , goal , start)

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

        self.current = None
        self.openNodes = []
        self.nodes=[]


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

        print(img.shape)
        
        self.g_map = self.create_distance_map( img.shape , start  )
        self.h_map = self.create_distance_map( img.shape , goal  )
        f = self.g_map + self.h_map
        self.f_map = (f[:,:,0]**2 + f[:,:,1]**2)**0.5
        print("f" , self.f_map.shape)
        self.sort_idx = np.argsort(self.f_map) # (525, 600)
        print("self.sort_idx" , self.sort_idx.shape)

        #while(1):
            # TODO: A Star Algorithm            
            #break
        for idx in self.sort_idx:
            #print("idx" , )
            if(idx != goal):
                #self._search()
                pass
            else:
                print("idx == goal")
                break
            pass


        self.goal_node = (int(200), int(300))
        self.parent[self.goal_node] = self.goal_node
        # Extract path
        path = []
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
