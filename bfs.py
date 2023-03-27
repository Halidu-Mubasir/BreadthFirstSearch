"""assign1_starter.py: Starter code for Assignment 1: Implementation of BFS."""
__author__ = "Mubasir Halidu"


class Problem:
    """ This class represents a generic Problem to be solved by search."""

    def __init__(self, init_state, goal_state):
        self.init_state = init_state
        self.goal_state = goal_state

    def __str__(self):
        return (type(self).__name__ + ": Init state=" + str(self.init_state) +
                ", goal state=" + str(self.goal_state))

    def goal_test(self, state):
        return False

    def actions(self, state):
        return None, None


class TwoDNavProb(Problem):
    """ This class represents the prolem of having a robot navigate
        in a 2D world with obstacles.
    """

    def __init__(self, init_state, goal_state, grid):
        super().__init__(init_state, goal_state)
        self.grid = grid

    def goal_test(self, state):
        return (state[0] == self.goal_state[0] and
                state[1] == self.goal_state[1])

    def actions(self, state):
        row = state[0]
        col = state[1]
        actions = []
        succ_states = []
        # U action results in (row-1, col)
        if (row-1 >= 0 and self.grid[row-1][col] == ' '):
            actions.append("U")
            succ_states.append((row-1, col))
        # D action results in (row+1, col)
        if (row+1 < len(self.grid) and self.grid[row+1][col] == ' '):
            actions.append("D")
            succ_states.append((row+1, col))
        # R action results in (row, col+1)
        if (col+1 < len(self.grid[0]) and self.grid[row][col+1] == ' '):
            actions.append("R")
            succ_states.append((row, col+1))
        # L action results in (row, col-1)
        if (col-1 >= 0 and self.grid[row][col-1] == ' '):
            actions.append("L")
            succ_states.append((row, col-1))

        # UL action results in (row-1, col-1)
        if (row-1 >= 0 and col-1 >= 0 and self.grid[row-1][col-1] == ' '):
            actions.append("UL")
            succ_states.append((row-1, col-1))

        # UR action results in (row-1, col+1)
        if (row-1 >= 0 and col+1 < len(self.grid[0]) and self.grid[row-1][col+1] == ' '):
            actions.append("UR")
            succ_states.append((row-1, col+1))

        # DL action results in (row+1, col-1)
        if (row+1 < len(self.grid) and col-1 >= 0 and self.grid[row+1][col-1] == ' '):
            actions.append("DL")
            succ_states.append((row+1, col-1))

        # DR action results in (row+1, col+1)
        if (row+1 < len(self.grid) and col+1 < len(self.grid[0]) and self.grid[row+1][col+1] == ' '):
            actions.append("DR")
            succ_states.append((row+1, col+1))

        return actions, succ_states

    def print_prob(self):
        print("__TwoDNavProb__")
        print("init_state =", self.init_state)
        print("goal_state =", self.goal_state)
        print("grid = ")
        for i in range(len(self.grid)):
            print("|", end="")
            for j in range(len(self.grid[i])):
                if (self.init_state[0] == i and self.init_state[1] == j):
                    print("S|", end='')
                elif (self.goal_state[0] == i and self.goal_state[1] == j):
                    print("G|", end='')
                else:
                    print(self.grid[i][j]+"|", end='')
            print()


class Node:
    """ This class represents a node in the search tree"""
    
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost

    def __str__(self):
        mystr = "Node with state=" + str(self.state)
        if (self.parent != None):
            mystr += (", parent=" + str(self.parent.state) +
                      ", action=" + str(self.action) +
                      ", path_cost=" + str(self.path_cost))
        return mystr

    # The purpose of this method is to enable two nodes  on the
    # frontier to be considered equal to each other if they represent
    # the same state (regardless of if they have different parent nodes)
    def __eq__(self, other):
        return (isinstance(other, Node) and
                self.state == other.state)

    def solution_path(self):
        current_state = self.parent
        state_path = [self.state]
        action_path = [self.action]
        while current_state != None:
            state_path.insert(0, current_state.state)
            if (current_state.action != None):
                action_path.insert(0, current_state.action)
            current_state = current_state.parent

        return state_path, action_path, self.path_cost


""" A function to implement the bread-first search algorithm"""

def bfs(problem):
    print("About to do BFS on problem: ", problem)
    node = Node(problem.init_state)
    number_of_nodes_processed = 0

    if problem.goal_test(node.state):
        print("The Number of Nodes Processed is: ",
              number_of_nodes_processed)
        print("The overall length of the frontier is: ",
              len_frontier)
        return node.solution_path()

    frontier = [node]
    explored = set()
    len_frontier = 1

    while (len(frontier) > 0):
        node = frontier.pop(0)
        number_of_nodes_processed += 1
        explored.add(node.state)
        print("Popped: ", node)
        actions, successors = problem.actions(node.state)
        print("Generated successor states: ", successors)
        for i in range(len(actions)):
            child = Node(successors[i], node, actions[i],
                         node.path_cost+1)
            if (child.state not in explored and
                    child not in frontier):
                if (problem.goal_test(child.state)):
                    print("Found a solution! ", child)
                    print("The Number of Nodes Processed is: ",
                          number_of_nodes_processed)
                    print("The overall length of the frontier is: ",
                          len_frontier)
                    return child.solution_path()
                frontier.append(child)
                len_frontier += 1

    return None  # failure


""" A function to implement the depth-first search algorithm"""

def dfs(problem):
    print("About to do DFS on problem: ", problem)
    node = Node(problem.init_state)
    number_of_nodes_processed = 0

    if problem.goal_test(node.state):
        print("The Number of Nodes Processed is: ",
              number_of_nodes_processed)
        print("The overall length of the frontier is: ",
              len_frontier)
        return node.solution_path()

    frontier = [node]
    explored = set()
    len_frontier = 1

    while (len(frontier) > 0):
        node = frontier.pop()
        number_of_nodes_processed += 1
        explored.add(node.state)
        print("Popped: ", node)
        actions, successors = problem.actions(node.state)
        print("Generated successor states: ", successors)
        for i in range(len(actions)):
            child = Node(successors[i], node, actions[i],
                         node.path_cost+1)
            if (child.state not in explored and
                    child not in frontier):
                if (problem.goal_test(child.state)):
                    print("Found a solution! ", child)
                    print("The Number of Nodes Processed is: ",
                          number_of_nodes_processed)
                    print("The overall length of the frontier is: ",
                          len_frontier)
                    return child.solution_path()
                frontier.append(child)
                len_frontier += 1

    return None  # failure


if __name__ == "__main__":

    print("Instantiating a TwoDNavProb")
    grid_1 = [[' ', ' ', ' ', ' ', 'X'],
              [' ', 'X', ' ', ' ', ' '],
              [' ', ' ', 'X', 'X', ' '],
              [' ', ' ', ' ', ' ', ' ']]

    grid_2 = [[' ', ' ', ' ', ' ', 'X', ' ', ' ', ' '],
              [' ', ' ', ' ', 'X', ' ', ' ', ' ', ' '],
              [' ', ' ', ' ', ' ', ' ', ' ', 'X', ' '],
              [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
              [' ', ' ', 'X', ' ', 'X', ' ', ' ', ' '],
              [' ', 'X', ' ', ' ', ' ', ' ', ' ', ' '],
              [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']]

    grid_3 = [[' ', ' ', ' ', ' ', 'X', ' '],
              [' ', ' ', ' ', 'X', ' ', ' '],
              [' ', 'X', ' ', ' ', ' ', ' '],
              [' ', ' ', ' ', 'X', ' ', ' '],
              [' ', ' ', 'X', ' ', 'X', ' '],
              [' ', 'X', ' ', ' ', ' ', ' ']]
    myProb_1 = TwoDNavProb((0, 3), (3, 2), grid_1)
    myProb_1.print_prob()
    print()
    print()

    # run breadth-first search on the instantiated problem
    solution = bfs(myProb_1)
    print("BFS returned", solution)
    print()
    print()

    # run depth-first search on the instantiated problem
    solution = dfs(myProb_1)
    print("DFS returned", solution)

    myProb_2 = TwoDNavProb((1, 4), (6, 7), grid_2)
    myProb_2.print_prob()
    print()
    print()

    # run breadth-first search on the instantiated problem
    solution = bfs(myProb_2)
    print("BFS returned", solution)
    print()
    print()

    # run depth-first search on the instantiated problem
    solution = dfs(myProb_2)
    print("DFS returned", solution)

    myProb_3 = TwoDNavProb((1, 4), (5, 2), grid_3)
    myProb_3.print_prob()
    print()
    print()

    # run breadth-first search on the instantiated problem
    solution = bfs(myProb_3)
    print("BFS returned", solution)
    print()
    print()

    # run depth-first search on the instantiated problem
    solution = dfs(myProb_3)
    print("DFS returned", solution)
