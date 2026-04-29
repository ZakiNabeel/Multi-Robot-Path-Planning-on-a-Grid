from collections import deque

# =============================================================================
# PART 1: READING THE INPUT FILE
# =============================================================================

def read_input():

    #Extracting Every Line from Input.txt
    f= open("exampleInput.txt", "r")
    lines = f.readlines()
    f.close()

    for i in range(len(lines)):
        lines[i] = lines[i].rstrip("\n")
    
    # Using idx to keeping track of line number.
    idx = 0

    #Grid Size (Extracting First Line)
    firstLine = lines[idx].split()
    N = int(firstLine[0])
    M = int(firstLine[1])
    idx+=1

    #Reading Grid
    grid ={}
    
    for rowIndex in range(N):
        rowText = lines[idx]
        idx+=1

        #y coordinate is 0 at bottom left
        y = N-1-rowIndex

        for x in range(len(rowText)):
            character = rowText[x]
            grid[(x,y)] = character
    
    #Number of Robots
    numberOfRobots = int(lines[idx])
    idx+=1

    #Now we need to store the data of each robot as well
    Robots = []

    for i in range(numberOfRobots):
        #Exctrating info from first line
        robotInfo = lines[idx].split()
        robotId = int(robotInfo[0])
        robotPriority = int(robotInfo[1])
        robotEnergy = int(robotInfo[2])
        idx+=1

        #Starting coordinates line
        startingInfo = lines[idx].split()
        xCoordinate = int(startingInfo[0])
        yCoordinate = int(startingInfo[1])
        idx+=1

        #Goal coordinates line
        finalInfo = lines[idx].split()
        fxCoordinate = int(finalInfo[0])
        fyCoordinate = int(finalInfo[1])
        idx+=1

        #Number of CheckPoints
        numberOfCheckpoints = int(lines[idx])
        idx+=1

        #Saving checkpoints info
        checkpointsList = []
        for checkpoints in range(numberOfCheckpoints):
            checkpointInfo = lines[idx].split()
            cxCoordinate = int(checkpointInfo[0])
            cyCoordinate = int(checkpointInfo[1])
            checkpointsList.append((cxCoordinate, cyCoordinate))
            idx+=1
        
        robot = {
            "id" : robotId,
            "priority" : robotPriority,
            "energy" : robotEnergy,
            "start" : (xCoordinate, yCoordinate),
            "goal" : (fxCoordinate, fyCoordinate),
            "checkPoints" :checkpointsList
        }

        Robots.append(robot)

    return N, M, grid, Robots

# =============================================================================
# PART 2: FIGURING OUT WHERE A ROBOT CAN MOVE
# =============================================================================

def getNeighbours(x, y, grid, N, M):

    currentCell = grid[(x,y)]

    neighbors = []
   
   #One Way Cells
    if currentCell == "^":   # UP
        nx, ny = x, y + 1
        if ny < N and grid[(nx, ny)] != "X":
            neighbors.append((nx, ny))

    elif currentCell == "v": # DOWN
        nx, ny = x, y - 1
        if ny >= 0 and grid[(nx, ny)] != "X":
            neighbors.append((nx, ny))

    elif currentCell == "<": # LEFT
        nx, ny = x - 1, y
        if nx >= 0 and grid[(nx, ny)] != "X":
            neighbors.append((nx, ny))

    elif currentCell == ">": # RIGHT
        nx, ny = x + 1, y
        if nx < M and grid[(nx, ny)] != "X":
            neighbors.append((nx, ny))

    #4 Possibilities + Wait possibility
    else:
        # WAIT
        neighbors.append((x, y))

        # UP
        if y + 1 < N and grid[(x, y + 1)] != "X":
            neighbors.append((x, y + 1))

        # DOWN
        if y - 1 >= 0 and grid[(x, y - 1)] != "X":
            neighbors.append((x, y - 1))

        # LEFT
        if x - 1 >= 0 and grid[(x - 1, y)] != "X":
            neighbors.append((x - 1, y))

        # RIGHT
        if x + 1 < M and grid[(x + 1, y)] != "X":
            neighbors.append((x + 1, y))

    return neighbors

# =============================================================================
# PART 3: BFS — finding the shortest path for one robot
# =============================================================================

def bfs(robot, grid, N, M, reservations):

    #initially extracting important info of a robot
    start = robot["start"]
    goal = robot["goal"]
    checkPointsInfo = robot["checkPoints"]
    energy = robot["energy"]
    K = len(checkPointsInfo)

    #return if start is already a goal state
    if start == goal and K==0:
        return [start]
    
    queue = deque()
    startState = (start[0], start[1], 0, 0) #start position, number of checkpoints visited, time
    queue.append(startState)

    #for tracing the path we have cameFrom ds
    cameFrom ={}
    cameFrom[startState] = None

    while len(queue) >0:
        #Current Sate
        currentState = queue.popleft()
        
        #Info of Current State
        xPos = currentState[0]
        yPos = currentState[1]
        checkpointIndexNeeded = currentState[2]
        time = currentState[3]

        newTime = time+1

        # If this move would use more energy than allowed, skip it      
        if newTime > energy:
            continue

        for (nx, ny) in getNeighbours(xPos, yPos, grid, N, M):

            #checking if we have a higher prioirty robot at (nx, ny) at newTime
            if (nx, ny, newTime) in reservations:
                continue

            # Check if visiting (nx, ny) completes the next checkpoint
            newCpIndex = checkpointIndexNeeded    # assuming no checkpoint is hit

            if checkpointIndexNeeded < K:
                nextCheckpoint = checkPointsInfo[checkpointIndexNeeded]

                if (nx, ny) == nextCheckpoint:
                    newCpIndex = checkpointIndexNeeded+1

            newState = (nx, ny, newCpIndex, newTime)

            #if we had already vsiited nextState
            if newState in cameFrom:
                continue

            #Tracing the path
            cameFrom[newState] = currentState

            #if we reached goal
            if newCpIndex  == K and (nx, ny) == goal:
                
                #tracing path
                path = []

                state = newState
                while state is not None:
                    path.append((state[0], state[1]))
                    state = cameFrom[state]

                path.reverse()
                return path
            
            #if not reach at goal
            queue.append(newState)

    #no path found
    return None


# =============================================================================
# PART 4: PLAN ALL ROBOTS IN PRIORITY ORDER
# =============================================================================

def plan_all_robots(N, M, grid, robots):

    # Sort robots so the highest priority goes first
    # We do this by sorting in DESCENDING order of priority
    sortedRobots = []

    sortedRobots = sorted(robots, key=lambda r: r["priority"], reverse=True)


    # 'reservations' is a set of (x, y, time) cells that are already taken
    # by higher-priority robots. Lower-priority robots cannot go there.
    reservations = set()

    # 'results' stores the final path for each robot, keyed by robot ID
    results = {}

    for robot in sortedRobots:

        print(f"Planning path for Robot {robot['id']} (priority {robot['priority']})...")

        path = bfs(robot, grid, N, M, reservations)

        # No valid path found for this robot       
        if path is None:
            results[robot["id"]] = None
            print(f"  → No path found!")
        else:
            results[robot["id"]] = path
            print(f"  → Path found! Length = {len(path) - 1} steps")

            # Add this robot's path to reservations so future robots avoid it
            for t in range(len(path)):
                x, y = path[t]
                reservations.add((x, y, t))

            # After the robot reaches its goal, it stays there forever.
            # Block that cell for future time steps too.
            last_x, last_y = path[-1]
            last_t = len(path) - 1
            for extra_t in range(last_t + 1, last_t + N * M + 1):
                reservations.add((last_x, last_y, extra_t))

    return results

# =============================================================================
# PART 5: WRITING THE OUTPUT FILE
# =============================================================================

def write_output(robots, results):

    # Sort robots by ID for output (smallest ID first)
    sorted_by_id = []
    remaining = list(robots)
    while len(remaining) > 0:
        smallest = remaining[0]
        for r in remaining:
            if r["id"] < smallest["id"]:
                smallest = r
        sorted_by_id.append(smallest)
        remaining.remove(smallest)

    f = open("output.txt", "w")

    for i in range(len(sorted_by_id)):
        robot = sorted_by_id[i]
        rid   = robot["id"]
        path  = results[rid]

        if path is None:
            f.write("Error: No valid path found for Robot " + str(rid) + "\n")
        else:
            T = len(path) - 1   # number of moves = path length minus 1
            E = T               # energy used = number of moves

            # Build the path string: "(0,0) -> (1,0) -> ..."
            path_string = ""
            for j in range(len(path)):
                x, y = path[j]
                path_string = path_string + "(" + str(x) + "," + str(y) + ")"
                if j < len(path) - 1:
                    path_string = path_string + " -> "

            f.write("Robot " + str(rid) + ":\n")
            f.write("Path: " + path_string + "\n")
            f.write("Total Time: " + str(T) + "\n")
            f.write("Total Energy: " + str(E) + "\n")

        # Add a blank line between robots (but not after the last one)
        if i < len(sorted_by_id) - 1:
            f.write("\n")

    f.close()
    print("Output written to output.txt")


# =============================================================================
# PART 6: MAIN
# =============================================================================

# Read everything from input.txt
N, M, grid, robots = read_input()

# Plan paths for all robots
results = plan_all_robots(N, M, grid, robots)

# Write results to output.txt
write_output(robots, results)



