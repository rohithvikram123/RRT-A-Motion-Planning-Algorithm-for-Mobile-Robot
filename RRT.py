import matplotlib.pyplot as plt
import numpy as np
import math
import time

class Point(object):

    def __init__(self, coord, cost, parent):
        self.coord = coord
        self.cost = cost
        self.parent = parent

    def CostInitialise(self, cost):
        self.cost = cost

    def NodeParent(self, parent):
        self.parent = parent

    def CoordEst(self):
        return self.coord

    def CostSet(self):
        return self.cost

    def Parent_Set(self):
        return self.parent


def Eucledian_Distance(pt_1, pt_2):
    return np.sqrt((pt_1[0]-pt_2[0])**2 + (pt_1[1]-pt_2[1])**2)

def steer(pt_1, pt_2, value, Threshold):
    pt_new = [0, 0]
    Dist_q1_to_q2 = np.sqrt((pt_1[0]-pt_2[0])**2 + (pt_1[1]-pt_2[1])**2)
    if Threshold < value:
        pt_new[0] = pt_2[0] + ((pt_1[0] - pt_2[0]) * Threshold) / Dist_q1_to_q2
        pt_new[1] = pt_2[1] + ((pt_1[1] - pt_2[1]) * Threshold) / Dist_q1_to_q2
    else:
        pt_new[0] = pt_1[0]
        pt_new[1] = pt_1[1]
    return Point(pt_new, 0, 0)

def LineCheck(l, m, r):
    if (m[0] <= max(l[0], r[0])) and (m[0] >= min(l[0], r[0])) and \
       (m[1] <= max(l[1], r[1])) and (m[1] >= min(l[1], r[1])):
        return True
    return False


def Intersection_Detection(l1, m1, l2, m2):
    def orientation(l, m, r):
        diff = (m[1] - l[1]) * (r[0] - m[0]) - (m[0] - l[0]) * (r[1] - m[1])
        if diff == 0:
            return 0
        return 1 if diff > 0 else 2

    # check if two line segments intersect
    orient_1 = orientation(l1, m1, l2)
    orient_2 = orientation(l1, m1, m2)
    orient_3 = orientation(l2, m2, l1)
    orient_4 = orientation(l2, m2, m1)
    
    if (orient_1 != orient_2) and (orient_3 != orient_4):
        return True
    
    if (orient_1 == 0) and LineCheck(l1, l2, m1):
        return True
    
    if (orient_2 == 0) and LineCheck(l1, m2, m1):
        return True
    
    if (orient_3 == 0) and LineCheck(l2, l1, m2):
        return True
    
    if (orient_4 == 0) and LineCheck(l2, m1, m2):
        return True
    
    return False


def Obstacle_Collision(m1, m2, obstacle_points):
    for obs_array in obstacle_points:
        for i in range(len(obs_array)):
            l1 = obs_array[i]
            l2 = obs_array[(i + 1) % len(obs_array)]
            if Intersection_Detection(m1, m2, l1, l2):
                return False
    return True

def User_Inputs_Start():
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        
        if((x>=0) and (x<=1000)) and (y>=0) and (y<=1000):
            start_node = (x,y)
            return start_node
        else:
            print("The Entered Start Node is in obstacle space")


#Getting User Input for the Goal Node from the user
def User_Inputs_Goal():
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        
        #goal_node = (x,y)
        if((x>=0) and (x<=1000)) and (y>=0) and (y<=1000):
            goal_node=(x,y)
            return goal_node
        else:
            print("The Entered Goal Node is in obstacle space")


def main():
    # PARAMETERS
    threshold = 20
    num_of_Node_pts = 3000
    X_limit = 1000
    Y_limit = 1000
    sx,sy = User_Inputs_Start()
    gx,gy = User_Inputs_Goal()

    startNode = Point([sx, sy], 0, 0)
    goalNode = Point([gx, gy], 0, 0)

    Node_s = []
    Node_s.append(startNode)
    EnvironmentLimits = [0, X_limit, 0, Y_limit]
    # IF YOU WANT TO USE YOUR OWN OBSTACLE BY SELECTING WITH MOUSE / UNCOMMENT HERE
    # obstacle_points = build_arena(number_of_obstacle, arena_limits)

    # IF YOU WANT TO USE SAME OBSTACLES / UNCOMMENT HERE
    Obs_Pts = []
    #Dense environment
    # First row
    Obs_Pts.append([(100,100),(200,100),(200,200),(100,200)])
    Obs_Pts.append([(300,100),(400,100),(400,200),(300,200)])
    Obs_Pts.append([(500,100),(600,100),(600,200),(500,200)])
    Obs_Pts.append([(700,100),(800,100),(800,200),(700,200)])
    Obs_Pts.append([(900,100),(1000,100),(1000,200),(900,200)])
    
    # Seconf row
    Obs_Pts.append([(0,300),(100,300),(100,400),(0,400)])
    Obs_Pts.append([(200,300),(300,300),(300,400),(200,400)])
    Obs_Pts.append([(400,300),(500,300),(500,400),(400,400)])
    Obs_Pts.append([(600,300),(700,300),(700,400),(600,400)])
    Obs_Pts.append([(800,300),(900,300),(900,400),(800,400)])
    
    # Third row
    Obs_Pts.append([(100,500),(200,500),(200,600),(100,600)])
    Obs_Pts.append([(300,500),(400,500),(400,600),(300,600)])
    Obs_Pts.append([(500,500),(600,500),(600,600),(500,600)])
    Obs_Pts.append([(700,500),(800,500),(800,600),(700,600)])
    Obs_Pts.append([(900,500),(1000,500),(1000,600),(900,600)])
    
    # Fourth row
    Obs_Pts.append([(0,700),(100,700),(100,800),(0,800)])
    Obs_Pts.append([(200,700),(300,700),(300,800),(200,800)])
    Obs_Pts.append([(400,700),(500,700),(500,800),(400,800)])
    Obs_Pts.append([(600,700),(700,700),(700,800),(600,800)])
    Obs_Pts.append([(800,700),(900,700),(900,800),(800,800)])

    # Fifth row
    Obs_Pts.append([(100,900),(200,900),(200,1000),(100,1000)])
    Obs_Pts.append([(300,900),(400,900),(400,1000),(300,1000)])
    Obs_Pts.append([(500,900),(600,900),(600,1000),(500,1000)])
    Obs_Pts.append([(700,900),(800,900),(800,1000),(700,1000)])
    Obs_Pts.append([(900,900),(1000,900),(1000,1000),(900,1000)])

    
    # Less dense environment:

    # Obs_Pts.append([(100,100),(200,100),(200,200),(100,200)])
    # Obs_Pts.append([(300,100),(400,100),(400,200),(300,200)])
    # Obs_Pts.append([(500,100),(600,100),(600,200),(500,200)])
    # Obs_Pts.append([(700,100),(800,100),(800,200),(700,200)])
    # Obs_Pts.append([(900,100),(1000,100),(1000,200),(900,200)])
    
        
    # # Third row
    # Obs_Pts.append([(100,500),(200,500),(200,600),(100,600)])
    # Obs_Pts.append([(300,500),(400,500),(400,600),(300,600)])
    # Obs_Pts.append([(500,500),(600,500),(600,600),(500,600)])
    # Obs_Pts.append([(700,500),(800,500),(800,600),(700,600)])
    # Obs_Pts.append([(900,500),(1000,500),(1000,600),(900,600)])
    

    # # Fifth row
    # Obs_Pts.append([(100,900),(200,900),(200,1000),(100,1000)])
    # Obs_Pts.append([(300,900),(400,900),(400,1000),(300,1000)])
    # Obs_Pts.append([(500,900),(600,900),(600,1000),(500,1000)])
    # Obs_Pts.append([(700,900),(800,900),(800,1000),(700,1000)])
    # Obs_Pts.append([(900,900),(1000,900),(1000,1000),(900,1000)])

    for pts in Obs_Pts:
        plt.axis(EnvironmentLimits)
        polygon = plt.Polygon(pts)
        axes = plt.gca()
        axes.add_patch(polygon)
    init_Time= time.time()
    for i in range(num_of_Node_pts):
        print(i, '/', num_of_Node_pts)
        random_point = Point([math.floor(np.random.rand()*X_limit), math.floor(np.random.rand()*Y_limit)], 0, 0)
        for j in range(len(Node_s)):
            if Node_s[j].CoordEst() == goalNode.CoordEst():
                break
        ndist = []
        for j in range(len(Node_s)):
            temp = Eucledian_Distance(Node_s[j].CoordEst(), random_point.CoordEst())
            ndist.append(temp)
        minimum_index = ndist.index(min(ndist))
        minimum_value = min(ndist)
        near_point = Node_s[minimum_index]

        new_point = steer(random_point.CoordEst(), near_point.CoordEst(), minimum_value, threshold)
        if Obstacle_Collision(random_point.CoordEst(), near_point.CoordEst(), Obs_Pts):
            plt.plot([new_point.CoordEst()[0], near_point.CoordEst()[0]], [new_point.CoordEst()[1], near_point.CoordEst()[1]], color='blue')
            new_point.CostInitialise(Eucledian_Distance(new_point.CoordEst(), near_point.CoordEst()) + near_point.CostSet())

            minimum_point = near_point
            for k in range(len(Node_s)):
                if Node_s[k].CoordEst() == minimum_point.CoordEst():
                    new_point.NodeParent(k)
            Node_s.append(new_point)
    D = []
    for i in range(len(Node_s)):
        tmpdist = Eucledian_Distance(Node_s[i].CoordEst(), goalNode.CoordEst())
        D.append(tmpdist)
    minimum_goal_node_index = D.index(min(D))
    final_point = Node_s[minimum_goal_node_index]
    end_point = final_point
    goalNode.NodeParent(minimum_goal_node_index)
    Node_s.append(goalNode)

    total_cost = Node_s[end_point.Parent_Set()].CostSet()
    while end_point.Parent_Set() != 0:
        start = end_point.Parent_Set()
        plt.plot([end_point.CoordEst()[0], Node_s[start].CoordEst()[0]],
                 [end_point.CoordEst()[1], Node_s[start].CoordEst()[1]], color='red')
        end_point = Node_s[start]
    start = end_point.Parent_Set()
    plt.plot([end_point.CoordEst()[0], Node_s[start].CoordEst()[0]],
             [end_point.CoordEst()[1], Node_s[start].CoordEst()[1]], color='red')
    print('The Total Cost is: ', total_cost)
    end_time = time.time()
    print(end_time - init_Time, ' seconds')
    plt.show()

if __name__ == "__main__":
    main()