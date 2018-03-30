"""
Randamized Rapidly-Exploring Random Trees (RRT) 

revised from:

    https://github.com/AtsushiSakai/PythonRobotics
"""

import random, math

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT():
    def __init__(self, start, goal, obstacle_list,
                 sampling_area, expand_dist=1.0,
                 sample_rate=5):
        """ 
        @Args:
            start(list): start position [x, y]
            goal(list): goal position [x, y]
            obstacle_list(list): obstacle positions
                [[x1,y1,size1], [x2,y2,size2], ...]
            sampling_area(list): 
                random sampling area(usually = configuration space)
                [min, max]
            expand_dist(float): the tree will growth toward the nearest node of the goal,
                this value is how far the tree should growth 
            sample_rate(float): 
                the probability that we growth our tree toward the 
                nearest node of the goal (not sampling);
                so (1-sample_rate) is toward the nearst node of
                the sample point (sampling)
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.min_rand = sampling_area[0]
        self.max_rand = sampling_area[1]
        self.expand_dist = expand_dist
        self.obstacle_list = obstacle_list
        self.sample_rate = sample_rate

    def __collision_check(self, target_node, obstacle_list):
        for ox, oy, size in obstacle_list:
            #print(target_node)
            d = self.get_distance([ox, oy], target_node)
            if d <= size:
                return True # collision
        return False

    def get_distance(self, node1, node2):
        if type(node1) == Node:
            if type(node2) == Node:
                dx = node1.x - node2.x
                dy = node1.y - node2.y
            elif type(node2) == list:
                dx = node1.x - node2[0]
                dy = node1.y - node2[1]
        elif type(node2) == Node:
            if type(node1) == Node:
                dx = node1.x - node2.x
                dy = node1.y - node2.y
            elif type(node1) == list:
                dx = node1[0] - node2.x
                dy = node1[1] - node2.y                
        else:
            dx = node1[0] - node2[0]
            dy = node1[0] - node2[1]              
        d = math.sqrt(dx**2 + dy**2)        
        return d

    def get_nearest_node_index(self, target_node, node_list):
        dist_list = [(target_node[0]-node.x)**2 + (target_node[1]-node.y)**2 for node in node_list]
        index = dist_list.index(min(dist_list))
        return index

    def planning(self):
        # the root
        self.node_list = [self.start]
        while True:
            if random.randint(0, 100) > self.sample_rate:
                rand_point = [random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand)]
            else:
                rand_point = [self.goal.x, self.goal.y]

            # we sampled a new point, now we want to growth our tree
            # toward this new point by adding a new node
            nearest_index = self.get_nearest_node_index(rand_point, self.node_list)
            #print(nearest_index)
            nearest_node = self.node_list[nearest_index]
            # atan2(y, x)
            theta = math.atan2(rand_point[1]-nearest_node.y,
                               rand_point[0]-nearest_node.x)
            # now we can add new node
            new_node = Node(nearest_node.x+self.expand_dist*math.cos(theta), 
                            nearest_node.y+self.expand_dist*math.sin(theta))
            new_node.parent = nearest_node

            if self.__collision_check(new_node, self.obstacle_list):
                continue # collision, give up this node
            self.node_list.append(new_node)

            d = self.get_distance(new_node, self.goal)

            if d <= self.expand_dist:
                self.goal.parent = new_node
                print('Goal!!')
                break

            self.draw_graph(rand_point)

        # get the route from start to goal
        current_node = self.goal
        path = [(current_node.x, current_node.y)]
        while True:
            #print(current_node.x, current_node.y)
            current_node = current_node.parent
            path.append((current_node.x, current_node.y))
            if current_node.parent is None:
                break            
        return path

    def draw_graph(self, rand_point=None):
        import matplotlib.pyplot as plt
        plt.clf()
        if rand_point is not None:
            plt.plot(rand_point[0], rand_point[1], '^k')

        plt.plot(self.start.x, self.start.y, 'xr')
        plt.plot(self.goal.x, self.goal.y, 'xr')
        for node in self.node_list:
            # plot nodes beside start and goal
            if node.parent is not None:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], '-g')
        for ox, oy, size in self.obstacle_list:
            # markersize is "area", so if you want to double
            # the marker, you should input 4
            plt.plot(ox, oy, 'ok', markersize=size*20)
        # axis limits [xmin, xmax, ymin, ymax]
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.001)

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    #====Search Path with RRT====
    obstacleList = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]

    rrt=RRT(start=[0,0],goal=[5,10],sampling_area=[-2,15],obstacle_list=obstacleList)
    path=rrt.planning()

    # Draw final path
    rrt.draw_graph()
    plt.plot([x for (x,y) in path], [y for (x,y) in path],'-r')
    plt.grid(True)
    plt.pause(0.01)  # Need for Mac
    plt.show()    





