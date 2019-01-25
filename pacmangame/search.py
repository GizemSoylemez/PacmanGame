
class SearchProblem:
    """
    Bu sınıfta hiç bir şeyi değiştirmene gerek yok.
    """

    def getStartState(self):
        """
        Arama sorununun başlangıç ​​durumunu döndürür.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
           Yalnızca, durum geçerli bir hedef durum olduğunda True değerini döndürür.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
         state: Arama durumu
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
   Bu yöntem, belirli bir eylem dizisinin toplam maliyetini döndürür.
   actions=eylemler anlamında
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
   TinyMaze'yi çözen bir hareket dizisi döndürür.(küçük labirent)
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """Önce arama ağacındaki en derin düğümleri arayın."""

    loc_stack = Stack()
    visited_node = {}
    parent_child_map = {}
    direction_list = []

    start_node = problem.getStartState()
    parent_child_map[start_node] = []
    loc_stack.push(start_node)

    def traverse_path(parent_node):
        while True:
            map_row = parent_child_map[parent_node]
            if (len(map_row) == 2):
                parent_node = map_row[0]
                direction = map_row[1]
                direction_list.append(direction)
            else:
                break
        return direction_list

    while (loc_stack.isEmpty() == False):

        parent_node = loc_stack.pop()

        if (problem.isGoalState(parent_node)):
            pathlist = traverse_path(parent_node)
            pathlist.reverse()
            return pathlist

        elif (visited_node.has_key(parent_node) == False):
            visited_node[parent_node] = []
            sucessor_list = problem.getSuccessors(parent_node)
            no_of_child = len(sucessor_list)
            if (no_of_child > 0):
                temp = 0
                while (temp < no_of_child):
                    child_nodes = sucessor_list[temp]
                    child_state = child_nodes[0];
                    child_action = child_nodes[1];
                    if (visited_node.has_key(child_state) == False):
                        loc_stack.push(child_state)
                        parent_child_map[child_state] = [parent_node, child_action]
                    temp = temp + 1


def breadthFirstSearch(problem):
    """Önce arama ağacındaki en sığ düğümleri arayın."""

    loc_queue = Queue()
    visited_node = {}
    parent_child_map = {}
    direction_list = []

    start_node = problem.getStartState()
    parent_child_map[start_node] = []
    loc_queue.push(start_node)

    def traverse_path(parent_node):
        while True:
            map_row = parent_child_map[parent_node]
            if (len(map_row) == 2):
                parent_node = map_row[0]
                direction = map_row[1]
                direction_list.append(direction)
            else:
                break
        return direction_list

    while (loc_queue.isEmpty() == False):

        parent_node = loc_queue.pop()

        if (problem.isGoalState(parent_node)):
            pathlist = traverse_path(parent_node)
            pathlist.reverse()
            return pathlist

        elif (visited_node.has_key(parent_node) == False):
            visited_node[parent_node] = []
            sucessor_list = problem.getSuccessors(parent_node)
            no_of_child = len(sucessor_list)
            if (no_of_child > 0):
                temp = 0
                while (temp < no_of_child):
                    child_nodes = sucessor_list[temp]
                    child_state = child_nodes[0];
                    child_action = child_nodes[1];
                    if (visited_node.has_key(child_state) == False):
                        loc_queue.push(child_state)
                    if (parent_child_map.has_key(child_state) == False):
                        parent_child_map[child_state] = [parent_node, child_action]
                    temp = temp + 1


def uniformCostSearch(problem):
    loc_pqueue = PriorityQueue()
    visited_node = {}
    parent_child_map = {}
    direction_list = []
    path_cost = 0

    start_node = problem.getStartState()
    parent_child_map[start_node] = []
    loc_pqueue.push(start_node, path_cost)

    def traverse_path(parent_node):
        while True:
            map_row = parent_child_map[parent_node]
            if (len(map_row) == 3):
                parent_node = map_row[0]
                direction = map_row[1]
                direction_list.append(direction)
            else:
                break
        return direction_list

    while (loc_pqueue.isEmpty() == False):

        parent_node = loc_pqueue.pop()

        if (parent_node != problem.getStartState()):
            path_cost = parent_child_map[parent_node][2]

        if (problem.isGoalState(parent_node)):
            pathlist = traverse_path(parent_node)
            pathlist.reverse()
            return pathlist

        elif (visited_node.has_key(parent_node) == False):
            visited_node[parent_node] = []
            sucessor_list = problem.getSuccessors(parent_node)
            no_of_child = len(sucessor_list)
            if (no_of_child > 0):
                temp = 0
                while (temp < no_of_child):
                    child_nodes = sucessor_list[temp]
                    child_state = child_nodes[0];
                    child_action = child_nodes[1];
                    child_cost = child_nodes[2];
                    gvalue = path_cost + child_cost
                    if (visited_node.has_key(child_state) == False):
                        loc_pqueue.push(child_state, gvalue)
                    if (parent_child_map.has_key(child_state) == False):
                        parent_child_map[child_state] = [parent_node, child_action, gvalue]
                    else:
                        if (child_state != start_node):
                            stored_cost = parent_child_map[child_state][2]
                            if (stored_cost > gvalue):
                                parent_child_map[child_state] = [parent_node, child_action, gvalue]
                    temp = temp + 1


def nullHeuristic(state, problem=None):
    """
    Sezgisel bir fonksiyon, mevcut durumdan maliyeti en yakın olana kadar tahmin eder.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """İlk olarak en düşük  maliyeti ve sezgisel olan düğümü arayalım."""

    loc_pqueue = PriorityQueue()
    visited_node = {}
    parent_child_map = {}
    direction_list = []
    path_cost = 0
    heuristic_value = 0

    start_node = problem.getStartState()
    parent_child_map[start_node] = []
    loc_pqueue.push(start_node, heuristic_value)

    def traverse_path(parent_node):
        temp = 0
        while True:
            '''print parent_node'''
            map_row = parent_child_map[parent_node]
            if (len(map_row) == 4):
                parent_node = map_row[0]
                direction = map_row[1]
                gvalue = map_row[2]
                fvalue = map_row[3]
                direction_list.append(direction)
                '''print "Gvalue = %d" % gvalue
                print fvalue'''
                '''print "Hueristic = %d" % (fvalue-gvalue)'''
                '''print "Admissible H = %d" % temp'''
                temp = temp + 1
            else:
                break
        return direction_list

    while (loc_pqueue.isEmpty() == False):

        parent_node = loc_pqueue.pop()

        if (parent_node != problem.getStartState()):
            path_cost = parent_child_map[parent_node][2]

        if (problem.isGoalState(parent_node)):
            pathlist = traverse_path(parent_node)
            pathlist.reverse()
            return pathlist

        elif (visited_node.has_key(parent_node) == False):
            visited_node[parent_node] = []
            sucessor_list = problem.getSuccessors(parent_node)
            no_of_child = len(sucessor_list)
            if (no_of_child > 0):
                temp = 0
                while (temp < no_of_child):
                    child_nodes = sucessor_list[temp]
                    child_state = child_nodes[0];
                    child_action = child_nodes[1];
                    child_cost = child_nodes[2];

                    heuristic_value = heuristic(child_state, problem)
                    gvalue = path_cost + child_cost
                    fvalue = gvalue + heuristic_value

                    if (visited_node.has_key(child_state) == False):
                        loc_pqueue.push(child_state, fvalue)
                    if (parent_child_map.has_key(child_state) == False):
                        parent_child_map[child_state] = [parent_node, child_action, gvalue, fvalue]
                    else:
                        if (child_state != start_node):
                            stored_fvalue = parent_child_map[child_state][3]
                            if (stored_fvalue > fvalue):
                                parent_child_map[child_state] = [parent_node, child_action, gvalue, fvalue]
                    temp = temp + 1


#Kısıtlamalar
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
