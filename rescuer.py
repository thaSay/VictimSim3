##  RESCUER AGENT
### @Author: Tacla (UTFPR)
### Demo of use of VictimSim

import os
import random
from abstract_agent import AbstractAgent
from physical_agent import PhysAgent
from abc import ABC, abstractmethod
from math import sqrt

## Class that defines the Rescuer Agent, which calculates the best route to save the victims
class Rescuer(AbstractAgent):
    def __init__(self, env, config_file):
        """ 
        @param env: a reference to an instance of the environment class
        @param config_file: the absolute path to the agent's config file"""

        super().__init__(env, config_file)

        # Specific initialization for the rescuer
        self.plan = []              # a list of planned actions
        self.rtime = self.TLIM      # for controlling the remaining time
        self.movements = self.body.getMovements() # the agent's movement

        # Starts in IDLE state.
        # It changes to ACTIVE when the map arrives
        self.body.set_state(PhysAgent.IDLE)
    
    def go_save_victims(self, walls, victims, grid):
        """ The explorer sends the map containing the walls and
        victims' location. The rescuer becomes ACTIVE. From now,
        the deliberate method is called by the environment"""
        self.walls = walls # all the walls that the Explorer found
        self.victims = victims # all the victims that the Explorer found
        self.grid = grid # the map that the explorer found
        self.__planner() # makes plan
        self.body.set_state(PhysAgent.ACTIVE)
         
    def __costH(self, position, destiny):
        """ The Heuristic defined was the length of the current position to the destiny position"""
        return sqrt((position[0]-destiny[0])**2+(position[1]-destiny[1])**2)

    def __Astar(self,pos,victim):
        """The IA decides what is the best way to get to base, using the heuristic and the cost to get there"""
        available = {}
        checked = {}

        available[pos] = {'g(n)':0,'h(n)':self.__costH(pos,victim),'father':None}

        #Get Path
        while True:
            current = None
            costF = 99999
            # Gets all paths available and selects the one with less cost
            for i in available.keys():
                if (available[i]['g(n)'] + available[i]['h(n)']) < costF:
                    costF = available[i]['g(n)'] + available[i]['h(n)']
                    current = i

            # get first available, adds to checked and removes from available
            checked[current] = available[current]
            del available[current]
            if current == victim:
                break

            # based in all possible movements
            for key in self.movements:
                opt = self.movements[key]
                nextPosOpt = (current[0]+opt[0],current[1]+opt[1])
                # ignores it if it is a wall or if it is already checked or if it is not in the map
                if nextPosOpt in checked.keys() or nextPosOpt in self.walls or not nextPosOpt in self.grid:
                    continue

                # gets the cost of the movement
                if opt[0] != 0 and opt[1] != 0:
                    movCost = self.COST_DIAG
                else:
                    movCost = self.COST_LINE
                
                # checks if it is not available or, if already is, if the cost is less than the one saved in available
                if nextPosOpt not in available.keys():
                    available[nextPosOpt] = {'g(n)':checked[current]['g(n)']+movCost,'h(n)':self.__costH(nextPosOpt,victim),'father':current}
                elif (checked[current]['g(n)']+movCost) < available[nextPosOpt]['g(n)']:
                    available[nextPosOpt]['g(n)'] = checked[current]['g(n)']+movCost
                    available[nextPosOpt]['father'] = current
        # Builds path
        current = victim
        path = []

        # Puts path of destiny to position, the reverse the list to get the correct path
        while not current == pos:
            newMov = (current[0] - checked[current]['father'][0], current[1] - checked[current]['father'][1])
            path.append(newMov)
            current = checked[current]['father']
        return {'path':list(reversed(path)),'cost':checked[victim]['g(n)']}
    
    def __planner(self):
        """ A private method that calculates the walk actions to rescue the
        victims, considering first the critical ones and the less far away, if
        one of the victims is already in the path, it saves it"""

        auxVictims = self.victims
        position = (0,0)
        # makes path until all the victims are put in the path
        while auxVictims:
            severity = 100
            distance = 99999
            path = []
            for victim in auxVictims.keys():
                _path = self.__Astar(position,victim)

                # based in the severity and the less far away, it selects the victim to be saved
                if int(auxVictims[victim][7]) < severity:
                    severity = int(auxVictims[victim][7])
                    distance = _path['cost']
                    choice = victim
                    path = _path['path']
                elif int(auxVictims[victim][7]) == severity and _path['cost'] < distance:
                    distance = _path['cost']
                    choice = victim
                    path = _path['path']
                    
            # the selected victim and all the victims in the path are removed, while the path is saved
            posAux = position
            for i in path:
                self.plan.append(i)

                posAux = (posAux[0]+i[0],posAux[1]+i[1])
                if posAux in auxVictims.keys():
                    del auxVictims[posAux]

            # the new position is now the position of the selected victim
            position = choice

        #Returns to base
        _path = self.__Astar(position,(0,0))
        for i in _path['path']:
            self.plan.append(i)
        
    def deliberate(self) -> bool:
        """ This is the choice of the next action. The simulator calls this
        method at each reasonning cycle if the agent is ACTIVE.
        Must be implemented in every agent
        @return True: there's one or more actions to do
        @return False: there's no more action to do """

        # No more actions to do
        if self.plan == []:  # empty list, no more actions to do
           return False

        # Takes the first action of the plan (walk action) and removes it from the plan
        dx, dy = self.plan.pop(0)

        # Walk - just one step per deliberation
        result = self.body.walk(dx, dy)

        # Rescue the victim at the current position
        if result == PhysAgent.EXECUTED:
            # check if there is a victim at the current position
            seq = self.body.check_for_victim()
            if seq >= 0:
                res = self.body.first_aid(seq) # True when rescued             

        return True

