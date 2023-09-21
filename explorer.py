## EXPLORER AGENT
### @Author: Tacla, UTFPR
### It walks randomly in the environment looking for victims.

import sys
import os
import random
from abstract_agent import AbstractAgent
from physical_agent import PhysAgent
from abc import ABC, abstractmethod
from math import sqrt


class Explorer(AbstractAgent):
    totalDiscoveredGrid = {(0, 0): {"backtrace": [], "type": ""}}
    allVictims = {}
    agentsFinished = 0
    def cluster(numAgents):
        k = numAgents
        centroid = []
        groupVictims = []
        pointK = {}
        cent = 0
        for position in Explorer.allVictims.keys():
            centroid.append(position)
            pointK[centroid[cent]] = cent
            print(position)
            cent += 1
            if cent >= k:
                break
        maxIt = 112345
        changed = 1
        numIt = 0
        print(maxIt)
        while(numIt < maxIt and changed == 1):
            changed = 0
            groupVictims = []
            for idx in range(k):
                groupVictims.append([])
            for point in Explorer.allVictims.keys():
                if(not point in pointK.keys()):
                    pointK[point] = -1
                min_centroid = 0
                min_dist = 112345
                for s in range(k):
                    if((point[0] - centroid[s][0]) ** 2 + (point[1] - centroid[s][1]) ** 2) < min_dist:
                        min_centroid = s
                        min_dist = (point[0] - centroid[s][0]) ** 2 + (point[1] - centroid[s][1]) ** 2
                groupVictims[min_centroid].append(point)
                if(pointK[point] != min_centroid):
                    changed = 1
                pointK[point] = min_centroid
            if changed == 0:
                break
            for i in range(0, k):
                medX = 0
                medY = 0
                nPoints = 0
                for point in groupVictims[i]:
                    medX += point[0]
                    medY += point[1]
                    nPoints += 1
                medX = medX/nPoints
                medY = medY/nPoints
                centroid[i] = (medX, medY)
            numIt += 1
        for i in range (0, k):
            group = {}
            print("1:")
            for point in groupVictims[i]:
                group[point] = Explorer.allVictims[point]
                print("    ")
                print(point)
            print(centroid[i])
            groupVictims.append(group)
        return groupVictims
    

    def completeMap(discoveredGrid, discoveredVictims):
        print("finished exp")
        for position in discoveredGrid.keys():
            if not position in Explorer.totalDiscoveredGrid.keys():
                Explorer.totalDiscoveredGrid[position] = discoveredGrid[position]
        for position in discoveredVictims.keys():
            if not position in Explorer.allVictims.keys():
                Explorer.allVictims[position] = discoveredVictims[position]
        if(Explorer.agentsFinished >= 4):
            Explorer.cluster(Explorer.agentsFinished)


    def __init__(self, env, config_file, resc, direction):
        """Explorer Constructor
        @param env Enviroment reference
        @config_file: the absolute path to the explorer's config file
        @param resc Rescuer reference
        """

        super().__init__(env, config_file)

        # Specific initialization for the rescuer
        self.resc = resc  # reference to the rescuer agent
        self.rtime = self.TLIM  # remaining time to explore
        self.grid = {(0, 0): {"backtrace": [], "type": ""}} # The map, which saves the last spot you came and if it's a wall or if it is okay
        self.position = (0, 0) # Initial position at base
        self.walls = [] # array of wall's position
        self.victims = {} # saves victim position and severity
        self.goingHome = False # states if it is going home or still exploring
        self.pathHome = {"path": [], "cost": 0} # saves the path to go home
        self.movements = self.body.getMovements() # gets movements from the phisical_agent

        # Since we had 4 agents, we decided to each one get a "square" of the map, focusing in different areas
        # Since the cost of line and diagonal can be different, it prioritizes the one with less cost
        if direction == 0:
            if self.COST_LINE<=self.COST_DIAG:
                self.nextDirections = [
                    self.movements["west"],
                    self.movements["north"],
                    self.movements["northWest"],
                    self.movements["east"],
                    self.movements["south"],
                    self.movements["northEast"],
                    self.movements["southEast"],
                    self.movements["southWest"]
                ]
            else:
                self.nextDirections = [
                    self.movements["northWest"],
                    self.movements["west"],
                    self.movements["north"],
                    self.movements["northEast"],
                    self.movements["southEast"],
                    self.movements["southWest"],
                    self.movements["east"],
                    self.movements["south"]
                ]
        elif direction == 1:
            if self.COST_LINE<=self.COST_DIAG:
                self.nextDirections = [
                    self.movements["north"],
                    self.movements["east"],
                    self.movements["northEast"],
                    self.movements["south"],
                    self.movements["west"],
                    self.movements["southEast"],
                    self.movements["southWest"],
                    self.movements["northWest"]
                ]
            else:
                self.nextDirections = [
                    self.movements["northEast"],
                    self.movements["north"],
                    self.movements["east"],
                    self.movements["southEast"],
                    self.movements["southWest"],
                    self.movements["northWest"],
                    self.movements["south"],
                    self.movements["west"]
                ]
        elif direction == 2:
            if self.COST_LINE<=self.COST_DIAG:
                self.nextDirections = [
                    self.movements["east"],
                    self.movements["south"],
                    self.movements["southEast"],
                    self.movements["west"],
                    self.movements["north"],
                    self.movements["southWest"],
                    self.movements["northWest"],
                    self.movements["northEast"]
                ]
            else:
                self.nextDirections = [
                    self.movements["southEast"],
                    self.movements["east"],
                    self.movements["south"],
                    self.movements["southWest"],
                    self.movements["northWest"],
                    self.movements["northEast"],
                    self.movements["west"],
                    self.movements["north"]
                ]
        elif direction == 3:
            if self.COST_LINE<=self.COST_DIAG:
                self.nextDirections = [
                    self.movements["south"],
                    self.movements["west"],
                    self.movements["southWest"],
                    self.movements["north"],
                    self.movements["east"],
                    self.movements["northWest"],
                    self.movements["northEast"],
                    self.movements["southEast"]
                ]
            else:
                self.nextDirections = [
                    self.movements["southWest"],
                    self.movements["south"],
                    self.movements["west"],
                    self.movements["northWest"],
                    self.movements["northEast"],
                    self.movements["southEast"],
                    self.movements["north"],
                    self.movements["east"]
                ]

    def __costH(self, position, destiny):
        """ The Heuristic defined was the length of the current position to the destiny position"""
        return sqrt((position[0] - destiny[0]) ** 2 + (position[1] - destiny[1]) ** 2)

    def chooseNextPoint(self):
        """It decides the next position based if it has an obstacle (wall or end of the map) and in the predefined positions"""
        obstacles = self.body.check_obstacles()
        obstaclesOrder = [
            (0, -1),
            (1, -1),
            (1, 0),
            (1, 1),
            (0, 1),
            (-1, 1),
            (-1, 0),
            (-1, -1)
        ]
        order = 0
        # if it is a wall, put it in the walls vector and in the grid, else ignores
        for i in obstacles:
            if i == 1 or i == 2:
                wallPos = (self.position[0] + obstaclesOrder[order][0], self.position[1] + obstaclesOrder[order][1])
                if not wallPos in self.grid.keys():
                    self.grid[wallPos] = {"backtrace": [], "type": "wall"}
                    self.walls.append(wallPos)
                """else:
                    self.grid[wallPos] = {"backtrace": [], "type" : "clear"}"""
            order += 1

        # if the direction is not in the grid vector, it is a possible path
        for direction in self.nextDirections:
            if (not (direction[0] + self.position[0], direction[1] + self.position[1]) in self.grid.keys()):
                return True, direction

        return False, (0, 0)

    def __Astar(self, position, dest):
        """The IA decides what is the best way to get to base, using the heuristic and the cost to get there"""
        available = {}
        checked = {}

        available[position] = {"g(n)": 0, "h(n)": self.__costH(position, dest), "father": None}
        #Get Path
        while True:
            current = None
            custoF = 2147483646
            # Gets all paths available and selects the one with less cost
            for i in available.keys():
                if (available[i]["g(n)"] + available[i]["h(n)"]) < custoF:
                    custoF = available[i]["g(n)"] + available[i]["h(n)"]
                    current = i

            # get first available, adds to checked and removes from available
            checked[current] = available[current]
            del available[current]
            if current == dest:
                break

            # based in all possible movements
            for opt in self.nextDirections:
                nextPosOpt = (current[0] + opt[0], current[1] + opt[1])
                # ignores it if it is a wall or if it is already checked or if it is not in the map
                if (nextPosOpt in checked.keys() or nextPosOpt in self.walls or not nextPosOpt in self.grid):
                    continue

                # gets the cost of the movement
                if opt[0] != 0 and opt[1] != 0:
                    movCost = self.COST_DIAG
                else:
                    movCost = self.COST_LINE

                # checks if it is not available or, if already is, if the cost is less than the one saved in available
                if nextPosOpt not in available.keys():
                    available[nextPosOpt] = {
                        "g(n)": checked[current]["g(n)"] + movCost,
                        "h(n)": self.__costH(nextPosOpt, dest),
                        "father": current,
                    }
                elif (checked[current]["g(n)"] + movCost) < available[nextPosOpt]["g(n)"]:
                    available[nextPosOpt]["g(n)"] = checked[current]["g(n)"] + movCost
                    available[nextPosOpt]["father"] = current

        # Builds path
        current = dest
        path = []

        # Puts path of destiny to position, the reverse the list to get the correct path
        while not current == position:
            newMov = (current[0] - checked[current]["father"][0], current[1] - checked[current]["father"][1])
            path.append(newMov)
            current = checked[current]["father"]
        return {"path": list(reversed(path)), "cost": checked[dest]["g(n)"]}

    def deliberate(self) -> bool:
        """The agent chooses the next action. The simulator calls this
        method at each cycle. Must be implemented in every agent"""

        # if it's not going home and the cost to go home is less than the maximum cost of going to another square and reading the vitals, it continues
        if not self.goingHome and self.pathHome["cost"] < (self.rtime - (self.COST_DIAG if self.COST_DIAG > self.COST_LINE else self.COST_LINE) - self.COST_READ):
            nextAvailable, movement = self.chooseNextPoint()

            #if it is not in a dead end (or has visited all the adjacents)
            if nextAvailable:
                nextPos = (self.position[0] + movement[0], self.position[1] + movement[1])

                # if position is not in map, saves it
                if not nextPos in self.grid.keys():
                    self.grid[nextPos] = {"backtrace": [self.position], "type": "ok"}

                # to do - verify if the cost is in two places
                result = self.body.walk(movement[0], movement[1])

                if movement[0] != 0 and movement[1] != 0:
                    self.rtime -= self.COST_DIAG
                else:
                    self.rtime -= self.COST_LINE

                # saves new position
                self.position = nextPos

                # check for victim returns -1 if there is no victim or the sequential
                # the sequential number of a found victim
                seq = self.body.check_for_victim()
                if seq >= 0:
                    vs = self.body.read_vital_signals(seq)
                    self.rtime -= self.COST_READ

                    self.victims[self.position] = vs
            else:
                # retreats back to the latest position to verify if all the other squares were visited
                nextPos = self.grid[self.position]["backtrace"][len(self.grid[self.position]["backtrace"]) - 1]
                del self.grid[self.position]["backtrace"][len(self.grid[self.position]["backtrace"]) - 1]

                dx = nextPos[0] - self.position[0]
                dy = nextPos[1] - self.position[1]

                self.position = nextPos

                self.body.walk(dx, dy)

                if dx != 0 and dy != 0:
                    self.rtime -= self.COST_DIAG
                else:
                    self.rtime -= self.COST_LINE
        else:
            # saves state that it is going home
            if not self.goingHome:
                self.goingHome = True
                print("Battery Low, returning to base")

            # has returned home
            if not self.pathHome["path"]:
                #self.resc.go_save_victims(self.walls, self.victims, self.grid)
                Explorer.agentsFinished += 1
                Explorer.completeMap(self.grid, self.victims)
                return False

            dx = self.pathHome["path"][0][0]
            dy = self.pathHome["path"][0][1]
            del self.pathHome["path"][0]

            if dx != 0 and dy != 0:
                self.rtime -= self.COST_DIAG
            else:
                self.rtime -= self.COST_LINE

            result = self.body.walk(dx, dy)

        # Update home's path
        if not self.goingHome:
            self.pathHome = self.__Astar(self.position, (0, 0))

        return True
