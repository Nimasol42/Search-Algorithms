from agent import AbstractSearchAgent
from collections import deque
import heapq

class BFSAgent(AbstractSearchAgent):
    def searching(self):
        print("BFS agent is searching...")
        goal, start = self.s_goal, self.s_start
        queue = deque([(start, [], 0)])  
        visited, seen, seen_teleports = [], set(), set()
        cost_map = {start: 0}
        while queue:
            current, path, current_cost = queue.popleft()
            visited.append(current)
            if current == goal:
                self.COST[self.s_goal] = current_cost  
                return path + [current], visited

            if current in self.teleports and current not in seen_teleports:# age teleport dashtim
                teleport_exit = self.teleports[current]
                seen_teleports.update([current, teleport_exit])
                queue.append((teleport_exit, path + [current],
                              current_cost + self.get_cost(current, teleport_exit)))

            
            for neighbor in self.get_neighbors(current):#barrasi tamam hamsayeh ha
                if neighbor not in seen:
                    seen.add(neighbor)
                    queue.append((neighbor, path + [current],
                                  current_cost + self.get_cost(current, neighbor)))
                    cost_map[neighbor] = current_cost + self.get_cost(current, neighbor)


        return [], visited

class BiIDDFSAgent(AbstractSearchAgent):
    def searching(self):
        
        limit, max_depth = 0, 350 #omgh avaliyeh va max omgh

        
        def dls(start, goal, depth, visited, order, parent, cost): #depth-limited search
            
            stack = [(start, 0)]
            parent[start], cost[start] = None, 0  
            while stack:
                node, d = stack.pop()
                visited.add(node)         
                order.append(node)       
                if node == goal:
                    return True          
                if d < depth:
                    
                    neighbors = list(self.get_neighbors(node))
                    if node in self.teleports:
                        neighbors.append(self.teleports[node])
                    for neighbor in neighbors:
                        if neighbor not in visited:
                            parent[neighbor] = node
                            cost[neighbor] = cost[node] + self.get_cost(node, neighbor)
                            stack.append((neighbor, d + 1))
            return False   

        
        while limit <= max_depth: #afzayesh tadrigi omgh
            
            f_visited, b_visited = set(), set() #az do taraf pish mirim
            f_order, b_order = [], []
            f_parent, b_parent = {}, {}
            f_cost, b_cost = {}, {}

            
            dls(self.s_start, self.s_goal, limit, f_visited, f_order, f_parent, f_cost)
            dls(self.s_goal, self.s_start, limit, b_visited, b_order, b_parent, b_cost)

            
            intersect = f_visited & b_visited #peyda kardane taghato do taraf
            if intersect:
                meet = intersect.pop()

                
                def build_chain(parent_map, node, reverse=False):
                    chain = []
                    while node is not None:
                        chain.append(node)
                        node = parent_map.get(node)
                    return chain[::-1] if reverse else chain

                
                forward_path = build_chain(f_parent, meet, True)  #masir az shoro ta taghato
                backward_path = build_chain(b_parent, b_parent.get(meet))  #az taghato ta hadag
                full_path = forward_path + backward_path

                
                self.COST[self.s_goal] = f_cost.get(meet, 0) + b_cost.get(meet, 0)
                
                for i in range(1, len(full_path)):
                    self.PARENT[full_path[i]] = full_path[i - 1]
                return full_path, f_order + b_order

            limit += 1

        return [], []


class AStarAgent(AbstractSearchAgent):
    def heuristic(self, a, b): 
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        D = 1             #amoodi ya ofoghi
        D2 = 1.41421356237 #movarab
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)
    
    def searching(self):
        
        start, goal = self.s_start, self.s_goal
        
        open_set = [(self.heuristic(start, goal), 0, start, [start])]
        visited, seen = [], set()
        
       
        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            if current in seen:
                continue
            seen.add(current)
            visited.append(current)

           
            if current == goal:
                self.COST = {goal: g}
                return path, visited

            
            for neighbor, cost in self.NEIGHBOR_COSTS.get(current, {}).items():
                if neighbor not in seen:
                    new_g = g + cost
                    new_f = new_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))
                    
        return [], visited

class UCSAgent(AbstractSearchAgent):
    def searching(self):
        print("UCS agent is searching...")
        start, goal = self.s_start, self.s_goal
        graph = self.NEIGHBOR_COSTS

        
        queue = [(0, start)] #safe (hzineh ta inja , node)
        cost_so_far = {start: 0}  
        self.PARENT[start] = None
        visited_nodes = []

        while queue:
            current_cost, current_node = heapq.heappop(queue)
            visited_nodes.append(current_node)
            if current_node == goal:
                self.COST = {goal: current_cost}
                return self.extract_path(), visited_nodes
            for neighbor, step_cost in graph.get(current_node, {}).items():
                new_cost = current_cost + step_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    self.PARENT[neighbor] = current_node
                    heapq.heappush(queue, (new_cost, neighbor))

        return [], visited_nodes