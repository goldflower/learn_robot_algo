"""
Randamized Rapidly-Exploring Random Trees (RRT) 

revised from:

    https://github.com/AtsushiSakai/PythonRobotics

Note:
    if the obstacles are small, the expand_dist should be small too,
    otherwise the path may cause collision, because we only check the 
    distance between sampling point to obstacle in this procedure
    for efficiency.
"""

import random, math

class Node():
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
    def __str__(self):
        return "Node({},{})".format(self.x, self.y)
    def __eq__(self, other):
        return (self.x == other.x and self.y == other.y)

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
        self.path = None
        self.path_length = None

    def __collision_check(self, target_node, obstacle_list):
        for ox, oy, size in obstacle_list:
            #print(target_node)
            d = self.get_distance(Node(ox, oy), target_node)
            if d <= size:
                return True # collision
        return False

    def get_path_distance(self, node1, node2):
        """ calculate the distance along self.path
        from node1 to node2 (node2 -> node2.parent -> ... -> node1)
        """
        current_node = node2
        dist = 0
        while True:
            if current_node == node1:
                break
            dist += self.get_distance(current_node, current_node.parent)
            current_node = current_node.parent
        return dist

    def get_distance(self, node1, node2):
        dx = node1.x - node2.x
        dy = node1.y - node2.y
               
        d = math.sqrt(dx**2 + dy**2)        
        return d

    def get_nearest_node_index(self, target_node, node_list):
        """ get the node index of node_list which is nearest node to target_node

        """
        dist_list = [(target_node[0]-node.x)**2 + (target_node[1]-node.y)**2 for node in node_list]
        index = dist_list.index(min(dist_list))
        return index

    def planning(self, draw = False):
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
            if draw:
                self.draw_graph(rand_point)

        self.update_path()

    def update_path(self):
        count = 1
        new_path = []
        current_node = self.goal
        new_path = [current_node]
        while current_node.parent:
            count += 1
            current_node = current_node.parent
            new_path.append(current_node)
        if new_path[-1] != self.start:
            count += 1
            new_path[-1].parent = self.start
            new_path.append(self.start)
        new_path.reverse()
        self.path_length = count
        self.path = new_path

    def smooth_the_path(self, path=None):
        """ try to make the path shorter
        the procedure:
        for i in range(max_iter):
            1. sample arbitrary node pair n1, n2
            2. if the distance(n1->...->n2) > distance(n1->n2):
                    if the lines are not intersect with any obstacle:
                        cut the nodes between n1, n2
        @Args:
            path(list): list of Nodes
            max_iter(int): the times to try to find better path

        """
        index = 0
        check_point = 0
        while True:
            for i in range(index, self.path_length-1):
                if self.check_line_intersect(self.path[index], self.path[i+1]):
                    continue
                origin_distance = self.get_path_distance(self.path[index], self.path[i+1])
                if self.get_distance(self.path[index], self.path[i+1]) < origin_distance:
                    self.path[i+1].parent = self.path[index]
                    check_point = i+1
            if index == check_point:
                index += 1
                check_point += 1
            else:
                index = check_point                
            if index == len(self.path):
                break
        self.update_path()            

    def check_line_intersect(self, n1, n2, obstacle_list=None):
        """ not really check intersection, just check the distance
        between the line between n1, n2 and the the center of obstacle
        @args:
            n1: node1
            n2: node2
            obstacle_list: obstacle list
            obstacle: when you want to check for a certain obstacle,
                      this will make the procedure don't check the whole list
        @return
            False if not intersected
            True if intersected
        """
        if obstacle_list is None:
            obstacle_list = self.obstacle_list
        a_list = []
        b_list = []
        # ax + y + b = 0
        if n1.x == n2.x:
            d_x1 = n1.x+0.001
            d_x2 = n1.x-0.001
            a1 = (n1.y - n2.y) / (d_x1 - n2.x)
            a2 = (n1.y - n2.y) / (d_x2 - n2.x)
            a_list.append(a1)
            a_list.append(a2)
            b_list.append(-(a1*d_x1 + n1.y))
            b_list.append(-(a1*d_x2 + n1.y))
        else:
            a = (n2.y - n1.y) / (n1.x - n2.x)
            b = -(a*n1.x + n1.y)


        for ox, oy, size in obstacle_list:
            radius = size/2
            if len(a_list) == 0:
                dist = abs(a*ox + oy + b) / math.sqrt(a**2 + 1)
                if dist < radius:
                    return True
            else:
                dist1 = abs(a_list[0]*ox + oy + b_list[0]) / math.sqrt(a_list[0]**2 + 1)
                dist2 = abs(a_list[1]*ox + oy + b_list[1]) / math.sqrt(a_list[1]**2 + 1)
                if dist1 < radius or dist2 < radius:
                    return True
        else:
            return False

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
            plt.plot(ox, oy, 'ok', markersize=(size-save_dist)*20)
        # axis limits [xmin, xmax, ymin, ymax]
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.001)

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    #====Search Path with RRT====
    obstacle_list = [
        (5, 5, 2),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2)
    ]  # [x,y,size]
    save_dist = 0.3 # in practice, we don't want to our path too close to any obstacle
    rrt=RRT(start=[0,0],goal=[5,10],sampling_area=[-2,15],obstacle_list=[(i,j,k+save_dist) for (i,j,k) in obstacle_list])
    rrt.planning()
    path = rrt.path
    rrt.draw_graph()
    plt.plot([n.x for n in path], [n.y for n in path],'-r')
    plt.grid(True)
    plt.pause(0.01)
    rrt.smooth_the_path(path)
    new_path = rrt.path
    plt.plot([n.x for n in new_path], [n.y for n in new_path],'-b')
    plt.pause(0.01)
    plt.show()



