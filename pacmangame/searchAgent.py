from game import Directions
from game import Agent
from game import Actions
from util import manhattanDistance
from search import breadthFirstSearch
import time
import search

class GoWestAgent(Agent):

    def getAction(self, state):
        "Aracı bir GameState (pacman.py dosyasında tanımlı) alıyor."
        if Directions.WEST in state.getLegalPacmanActions():
            return Directions.WEST
        else:
            return Directions.STOP

class SearchAgent(Agent):
    """
 Varsayılan olarak, bu aracı bulmak için DFS'yi bir PositionSearchProblem üzerinde çalıştırır.
    Fn için seçenekler şunları içerir:
      depthFirstSearch veya dfs
      breadthFirstSearch veya bfs
    Not: SearchAgent'ta hiçbir kodu DEĞİŞTİRMEMELİDİR
    """

    def __init__(self, fn='depthFirstSearch', prob='PositionSearchProblem', heuristic='nullHeuristic'):

        # Uyarı: Doğru fonksiyonları ve problemleri bulmak için bazı gelişmiş Python sihirleri kullanılır.
        if fn not in dir(search):
            raise AttributeError, fn + 'search.pyde bir arama işlevi değil.'
        func = getattr(search, fn)
        if 'heuristic' not in func.func_code.co_varnames:
            print('[SearchAgent] işlevi kullanmak' + fn)
            self.searchFunction = func
        else:
            if heuristic in globals().keys():
                heur = globals()[heuristic]
            elif heuristic in dir(search):
                heur = getattr(search, heuristic)
            else:
                raise AttributeError, heuristic + ' searchAgents.py veya search.pyde bir işlev değil.'
            print('[SearchAgent] using function %s and heuristic %s' % (fn, heuristic))

            # Not: Bu Python hilesi biraz arama algoritması ve sezgisel birleştirir
            self.searchFunction = lambda x: func(x, heuristic=heur)
        if prob not in globals().keys() or not prob.endswith('Problem'):
            raise AttributeError, prob + ' SearchAgents.py de bir arama sorunu türü değil.'
        self.searchType = globals()[prob]
        print('[SearchAgent] problem türünü kullanma' + prob)

    def registerInitialState(self, state):
        """
        Burada hedefe giden yolu seçiyoruz. Bu aşamada, ajan
        Hedefe giden yolu hesaplamalı ve yerel bir değişkende saklamalıdır.
        """
        if self.searchFunction == None: raise Exception, 'SearchAgent için hiçbir arama işlevi sağlanmadı'
        starttime = time.time()
        problem = self.searchType(state)  # Makes a new search problem
        self.actions = self.searchFunction(problem)  # Find a path
        totalCost = problem.getCostOfActions(self.actions)
        print('Yol toplam maliyeti ile bulundu %d in %.1f seconds' % (totalCost, time.time() - starttime))
        if '_expanded' in dir(problem): print('Arama düğümleri genişletildi: %d' % problem._expanded)

    def getAction(self, state):

        if 'actionIndex' not in dir(self): self.actionIndex = 0
        i = self.actionIndex
        self.actionIndex += 1
        if i < len(self.actions):
            return self.actions[i]
        else:
            return Directions.STOP


class PositionSearchProblem(search.SearchProblem):


    def __init__(self, gameState, costFn=lambda x: 1, goal=(1, 1), start=None, warn=True, visualize=True):
        """
       Başlangıç ​​ve hedefi depolar.
        gameState: Bir GameState nesnesi
        costFn: Bir arama durumundan (tuple) negatif olmayan bir sayıya bir işlev
        goal: gameState'deki bir konum
        """
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        if start != None: self.startState = start
        self.goal = goal
        self.costFn = costFn
        self.visualize = visualize
        if warn and (gameState.getNumFood() != 1 or not gameState.hasFood(*goal)):
            print
            'Warning: this does not look like a regular search maze'

        # Teşhir amaçlı
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def getStartState(self):
        return self.startState

    def isGoalState(self, state):
        isGoal = state == self.goal

        # Sadece görüntüleme amaçlı
        if isGoal and self.visualize:
            self._visitedlist.append(state)
            import __main__
            if '_display' in dir(__main__):
                if 'drawExpandedCells' in dir(__main__._display):  # @UndefinedVariable
                    __main__._display.drawExpandedCells(self._visitedlist)  # @UndefinedVariable

        return isGoal

    def getSuccessors(self, state):


        successors = []
        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextState = (nextx, nexty)
                cost = self.costFn(nextState)
                successors.append((nextState, action, cost))

       # Görüntüleme amaçlı defter tutma
        self._expanded += 1  # DO NOT CHANGE
        if state not in self._visited:
            self._visited[state] = True
            self._visitedlist.append(state)

        return successors

    def getCostOfActions(self, actions):

        if actions == None: return 999999
        x, y = self.getStartState()
        cost = 0
        for action in actions:
            # Check figure out the next state and see whether its' legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
            cost += self.costFn((x, y))
        return cost


class StayEastSearchAgent(SearchAgent):

    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: .5 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn, (1, 1), None, False)


class StayWestSearchAgent(SearchAgent):


    def __init__(self):
        self.searchFunction = search.uniformCostSearch
        costFn = lambda pos: 2 ** pos[0]
        self.searchType = lambda state: PositionSearchProblem(state, costFn)


def manhattanHeuristic(position, problem, info={}):
    """manhattan uzaklığı"""
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])


def euclideanHeuristic(position, problem, info={}):

    xy1 = position
    xy2 = problem.goal
    return ((xy1[0] - xy2[0]) ** 2 + (xy1[1] - xy2[1]) ** 2) ** 0.5

class CornersProblem(search.SearchProblem):

    def __init__(self, startingGameState):

        self.walls = startingGameState.getWalls()
        self.startingPosition = startingGameState.getPacmanPosition()
        top, right = self.walls.height - 2, self.walls.width - 2
        self.corners = ((1, 1), (1, top), (right, 1), (right, top))
        for corner in self.corners:
            if not startingGameState.hasFood(*corner):
                print
                'Uyarı: köşede yiyecek yok ' + str(corner)
        self._expanded = 0  # DEĞİŞTİRME; Genişletilmiş genişletilmiş düğüm sayısı

        "*** Kod burada ***"
        self.Corner1 = False
        self.Corner2 = False
        self.Corner3 = False
        self.Corner4 = False
        '''self.corner_visited = 0'''

    def getStartState(self):
        return self.startingPosition

    def isGoalState(self, state):
        """Bu arama durumunun sorunun bir amaç durumu olup olmadığını döndürür.        """
        if not (state == self.startingPosition):
            corner_visited = state[1]
            if (corner_visited == (True, True, True, True)):
                return state

    def getSuccessors(self, state):

        successors = []

        for action in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:

            if not (state == self.startingPosition):
                postition_state = state[0]
                self.Corner1 = state[1][0]
                self.Corner2 = state[1][1]
                self.Corner3 = state[1][2]
                self.Corner4 = state[1][3]
            else:
                postition_state = state
                self.Corner1 = False
                self.Corner2 = False
                self.Corner3 = False
                self.Corner4 = False

            x, y = postition_state
            dx, dy = Actions.directionToVector(action)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                cost = 1
                new_position = (nextx, nexty)
                if (new_position == self.corners[0]):
                    self.Corner1 = True
                if (new_position == self.corners[1]):
                    self.Corner2 = True
                if (new_position == self.corners[2]):
                    self.Corner3 = True
                if (new_position == self.corners[3]):
                    self.Corner4 = True
                nextState = (new_position, (self.Corner1, self.Corner2, self.Corner3, self.Corner4))
                successors.append((nextState, action, cost))

        self._expanded += 1
        return successors

    def getCostOfActions(self, actions):
        """maliyetini döndürür"""
        if actions == None: return 999999
        x, y = self.startingPosition
        for action in actions:
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]: return 999999
        return len(actions)


def cornersHeuristic(state, problem):
    """
    Tanımladığınız CornersProblem için bir sezgisel.
    Bu işlev her zaman alt sınırı olan bir sayıyı döndürmelidir. sorunun en kısa yolu olmalı(aynı zamanda tutarlı).
    """
    corners = problem.corners  # Bunlar köşe koordinatları
    walls = problem.walls  # labirent duvarlarıdır

    corner_unvisited = []
    total_distance = 0

    if not (state == problem.startingPosition):
        postition_state = state[0]
        if (state[1][0] == False):
            corner_unvisited.append(problem.corners[0])
        if (state[1][1] == False):
            corner_unvisited.append(problem.corners[1])
        if (state[1][2] == False):
            corner_unvisited.append(problem.corners[2])
        if (state[1][3] == False):
            corner_unvisited.append(problem.corners[3])
    else:
        postition_state = state
        corner_unvisited = list(problem.corners)

    if (len(corner_unvisited) == 0):
        return 0

    while (len(corner_unvisited) > 0):
        distance_to_all_corners = []
        for selected_corner in corner_unvisited:
            xy1 = postition_state
            xy2 = selected_corner
            distance_to_corner = abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
            distance_to_all_corners.append((distance_to_corner, selected_corner))

        closest_corner = min(distance_to_all_corners)
        corner_unvisited.remove(closest_corner[1])
        postition_state = closest_corner[1]
        total_distance = total_distance + closest_corner[0]

    return total_distance  # Default to trivial solution


class AStarCornersAgent(SearchAgent):


    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, cornersHeuristic)
        self.searchType = CornersProblem


class FoodSearchProblem:
    """
  foodGrid: Kalan yiyeceği belirten Doğru veya Yanlış olan bir Kılavuz
    """

    def __init__(self, startingGameState):
        self.start = (startingGameState.getPacmanPosition(), startingGameState.getFood())
        self.walls = startingGameState.getWalls()
        self.startingGameState = startingGameState
        self._expanded = 0  # DO NOT CHANGE
        self.heuristicInfo = {}  # A dictionary for the heuristic to store information

    def getStartState(self):
        return self.start

    def isGoalState(self, state):
        return state[1].count() == 0

    def getSuccessors(self, state):
        successors = []
        self._expanded += 1  # DO NOT CHANGE
        for direction in [Directions.NORTH, Directions.SOUTH, Directions.EAST, Directions.WEST]:
            x, y = state[0]
            dx, dy = Actions.directionToVector(direction)
            nextx, nexty = int(x + dx), int(y + dy)
            if not self.walls[nextx][nexty]:
                nextFood = state[1].copy()
                nextFood[nextx][nexty] = False
                successors.append((((nextx, nexty), nextFood), direction, 1))
        return successors

    def getCostOfActions(self, actions):
       """ Belirli bir eylem dizisinin maliyetini döndürür."""
        x, y = self.getStartState()[0]
        cost = 0
        for action in actions:
            # figure out the next state and see whether it's legal
            dx, dy = Actions.directionToVector(action)
            x, y = int(x + dx), int(y + dy)
            if self.walls[x][y]:
                return 999999
            cost += 1
        return cost


class AStarFoodSearchAgent(SearchAgent):


    def __init__(self):
        self.searchFunction = lambda prob: search.aStarSearch(prob, foodHeuristic)
        self.searchType = FoodSearchProblem


def foodHeuristic(state, problem):
    """
    A * kullanılıyorsa, daha kötü bir maliyet araştırması buluntusu olan bir çözüm bulur.
    """
    position, foodGrid = state
    hvalue = 0
    food_available = []
    total_distance = 0
    for i in range(0, foodGrid.width):
        for j in range(0, foodGrid.height):
            if (foodGrid[i][j] == True):
                food_location = (i, j)
                food_available.append(food_location)

    if (len(food_available) == 0):
        return 0

    max_distance = ((0, 0), (0, 0), 0)

    for current_food in food_available:
        for select_food in food_available:
            if (current_food == select_food):
                pass
            else:
                distance = manhattanDistance(current_food, select_food)
                if (max_distance[2] < distance):
                    max_distance = (current_food, select_food, distance)

    if (max_distance[0] == (0, 0) and max_distance[1] == (0, 0)):
        hvalue = manhattanDistance(position, food_available[0])
    else:
        d1 = manhattanDistance(position, max_distance[0])
        d2 = manhattanDistance(position, max_distance[1])
        hvalue = max_distance[2] + min(d1, d2)

    return hvalue


class ClosestDotSearchAgent(SearchAgent):
    """Bir dizi arama kullanarak tüm yiyecekleri ara"""

    def registerInitialState(self, state):
        self.actions = []
        currentState = state
        while (currentState.getFood().count() > 0):
            nextPathSegment = self.findPathToClosestDot(currentState)  # The missing piece
            self.actions += nextPathSegment
            for action in nextPathSegment:
                legal = currentState.getLegalActions()
                if action not in legal:
                    t = (str(action), str(currentState))
                    raise Exception, 'findPathToClosestDot returned an illegal move: %s!\n%s' % t
                currentState = currentState.generateSuccessor(0, action)
        self.actionIndex = 0
        print
        'Path found with cost %d.' % len(self.actions)

    def findPathToClosestDot(self, gameState):
        """
           En yakın noktaya giden bir yolu döndürür.
        """

        startPosition = gameState.getPacmanPosition()
        food = gameState.getFood()
        walls = gameState.getWalls()
        problem = AnyFoodSearchProblem(gameState)

        action_list = breadthFirstSearch(problem)

        return action_list


class AnyFoodSearchProblem(PositionSearchProblem):
    """Herhangi bir yemek için bir yol bulmak için bir arama problemi."""

    def __init__(self, gameState):
        "Bilgileri gameState'den depolar. Bunu değiştirmenize gerek yoktur."
        # Daha sonra başvurmak için yiyecekleri saklayın
        self.food = gameState.getFood()
        self.walls = gameState.getWalls()
        self.startState = gameState.getPacmanPosition()
        self.costFn = lambda x: 1
        self._visited, self._visitedlist, self._expanded = {}, [], 0  # DO NOT CHANGE

    def isGoalState(self, state):

        x, y = state
        foodGrid = self.food
        if (foodGrid[x][y] == True) or (foodGrid.count() == 0):
            return True


def mazeDistance(point1, point2, gameState):
    """
   Arama fonksiyonlarını kullanarak herhangi iki nokta arasındaki labirent mesafesini döndürür.
    """
    x1, y1 = point1
    x2, y2 = point2
    walls = gameState.getWalls()
    assert not walls[x1][y1], 'point1 is a wall: ' + str(point1)
    assert not walls[x2][y2], 'point2 is a wall: ' + str(point2)
    prob = PositionSearchProblem(gameState, start=point1, goal=point2, warn=False, visualize=False)
    return len(search.bfs(prob))
