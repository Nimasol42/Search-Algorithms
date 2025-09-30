import time

from env import Env
from plotting import Plotting
from implemented_agents import BFSAgent, BiIDDFSAgent, AStarAgent, UCSAgent

def run_algorithm(agent_class, start, goal, environment, euclidean_cost, FPS):#baraye inkeh hameye  algorithm ha ro ejra konim
    print(f"Running {agent_class.__name__}...")  
    agent = agent_class(start, goal, environment, euclidean_cost)
    start_time = time.time()
    path, visited = agent.searching()
    end_time = time.time()
    run_time = end_time - start_time
    print(f"{agent_class.__name__} completed in {run_time:.5f} seconds")  

    print("Visited nodes:", visited)  

    
    plot = Plotting(start, goal, environment, FPS, algorithm_name=agent_class.__name__)#esme algorithm ro ezafeh kardam
    plot.animation(path, visited, agent.COST)

    print(f"Finished animation for {agent_class.__name__}\n")  

def main():
    map_name = "default"  # Choose the map file
    use_random_teleports = True  # Change to True to use random teleports
    num_pairs = 7  # Number of random teleport gates if enabled
    FPS = 6000000  # Frames per second for animation

    start = (5, 5)  # Start position
    goal = (45, 25)  # Goal position
    euclidean_cost = True  # True to use Euclidean distance as cost

    environment = Env(map_name, use_random_teleports, num_pairs)
    
    agents = [BFSAgent, BiIDDFSAgent, AStarAgent, UCSAgent]

    
    for agent_class in agents:
        run_algorithm(agent_class, start, goal, environment, euclidean_cost, FPS)

if __name__ == "__main__":
    main()