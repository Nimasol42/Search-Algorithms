from agent import AbstractSearchAgent
from collections import deque
import heapq
import math

class BFSAgent(AbstractSearchAgent):
    def searching(self):
        queue = deque([self.s_start])  # shoru ba s_start dar saf
        self.PARENT[self.s_start] = None  
        self.COST[self.s_start] = 0  
        self.VISITED = {self.s_start}  

        while queue:
            current = queue.popleft()  # bardashtane node az saf
            if current == self.s_goal:
                return self.extract_path(), list(self.VISITED)
            for neighbor in self.get_neighbors(current):
                if neighbor not in self.VISITED:
                    self.VISITED.add(neighbor)
                    self.PARENT[neighbor] = current
                    self.COST[neighbor] = self.COST[current] + self.get_cost(current, neighbor)
                    queue.append(neighbor)  # ezafe kardan hamsayeh be saf
        return [], list(self.VISITED)

class BiIDDFSAgent(AbstractSearchAgent):#az har do tarafe start va goal DFS ba mahdoodiat omgh mizanim       
    def searching(self):
        if self.s_start == self.s_goal:
            self.PARENT[self.s_goal] = None
            self.COST[self.s_goal] = 0
            self.VISITED.add(self.s_start)
            return [self.s_start], list(self.VISITED)
        def dls(node, limit, visited, parent):
            visited.add(node)
            if limit <= 0:
                return
            for neighbor in self.get_neighbors(node):
                if neighbor not in visited:
                    parent[neighbor] = node
                    dls(neighbor, limit - 1, visited, parent)
        max_depth = self.Env.x_range * self.Env.y_range  # meghdar max omgh baraye search
        for depth in range(max_depth):
            forward_visited = set()
            forward_parent = {}
            dls(self.s_start, depth, forward_visited, forward_parent)
            backward_visited = set()
            backward_parent = {}
            dls(self.s_goal, depth, backward_visited, backward_parent)
            intersection = forward_visited.intersection(backward_visited)
            if intersection:
                meeting_node = intersection.pop()
                path_forward = []
                node = meeting_node
                while node is not None:
                    path_forward.append(node)
                    if node == self.s_start:
                        break
                    node = forward_parent.get(node)
                path_forward.reverse()
                path_backward = []
                node = meeting_node
                while node is not None:
                    path_backward.append(node)
                    if node == self.s_goal:
                        break
                    node = backward_parent.get(node)
                full_path = path_forward + path_backward[1:]
                self.COST = {}
                cost_so_far = 0
                for n in full_path:
                    self.COST[n] = cost_so_far
                    cost_so_far += 1
                self.PARENT = {}
                for i in range(1, len(full_path)):
                    self.PARENT[full_path[i]] = full_path[i - 1]
                self.PARENT[self.s_start] = None
                self.VISITED = forward_visited.union(backward_visited)
                return full_path, list(self.VISITED)
        return [], list(self.VISITED)


class AStarAgent(AbstractSearchAgent):
    def heuristic(self, s):
        dx = abs(s[0] - self.s_goal[0])
        dy = abs(s[1] - self.s_goal[1])
        # aval hadaghal harakat movarab ro mirim ke fasele ro dar zar , toll kam mikone
        # h = max(dx, dy) + (sqrt(2)-1)*min(dx, dy)
        return max(dx, dy) + (1.41421356 - 1) * min(dx, dy)

    def searching(self):
        open_set = []
        heapq.heappush(open_set, (0, self.s_start))  # shoru ba s_start dar open_set
        self.COST[self.s_start] = 0
        self.PARENT[self.s_start] = None
        self.VISITED = set()

        while open_set:
            current_priority, current = heapq.heappop(open_set)
            if current == self.s_goal:
                return self.extract_path(), list(self.VISITED)
            if current in self.VISITED:
                continue
            self.VISITED.add(current)
            for neighbor in self.get_neighbors(current):
                new_cost = self.COST[current] + self.get_cost(current, neighbor)
                if neighbor not in self.COST or new_cost < self.COST[neighbor]:
                    self.COST[neighbor] = new_cost
                    priority = new_cost + self.heuristic(neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    self.PARENT[neighbor] = current
        return [], list(self.VISITED)
class UCSAgent(AbstractSearchAgent):
    
    def searching(self):
        open_set = []
        heapq.heappush(open_set, (0, self.s_start))
        self.COST[self.s_start] = 0
        self.PARENT[self.s_start] = None
        self.VISITED = set()

        while open_set:
            cost, current = heapq.heappop(open_set)
            if current == self.s_goal:
                return self.extract_path(), list(self.VISITED)
            if current in self.VISITED:
                continue
            self.VISITED.add(current)
            for neighbor in self.get_neighbors(current):
                new_cost = self.COST[current] + self.get_cost(current, neighbor)
                if neighbor not in self.COST or new_cost < self.COST[neighbor]:
                    self.COST[neighbor] = new_cost
                    heapq.heappush(open_set, (new_cost, neighbor))
                    self.PARENT[neighbor] = current
        return [], list(self.VISITED)