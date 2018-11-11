# -*- coding: utf-8 -*-
"""
Created on Sat Nov 10 01:37:43 2018

@author: Aubrey
"""

# -*- coding: utf-8 -*-


import random
import numpy as np
from scipy.optimize import linear_sum_assignment as HungAlg
import queue
import tkinter as tk

class Agent:
    
    def __init__(self, coords, dims):
        self.startTime = 0
        self.vel = dims["unitDist"]//5
        self.dims = dims
        
        self.pos = coords
        self.goal = self.pos
        
        self.path = []
        self.pixPath = []
        self.pathsIndex = 0
        
        #px, py -> pixel coords
        self.px = (self.pos[0]) * self.dims["unitDist"] +  self.dims["buffer"]
        self.py = (self.pos[1])* self.dims["unitDist"] +  self.dims["buffer"]
        
        #booleann
        self.go = 0

    def mapPixelPath(self, path, t):
        self.path = path
        self.startTime = t
        origin = (self.dims["buffer"], self.dims["buffer"])
        self.pixPath = []
        
        #given node, path, need to convert to pixels for the gui
        for node in self.path:
            pixCoords = (origin[0] + self.dims["unitDist"] * node.coords[0],
                         origin[1] + self.dims["unitDist"] * node.coords[1])
            self.pixPath.append(pixCoords)        
        self.px = self.pixPath[0][0]
        self.py = self.pixPath[0][1]
        self.goal = self.path[len(self.path) - 1].coords
            
            
    def clearAssignments(self):    
        
        self.goal = self.pos
        self.startTime = 0
        self.path = []
        self.pixPath = []
        self.pathsIndex = 0
        self.px = self.pos[0] * self.dims["unitDist"] + self.dims["buffer"]
        self.py = self.pos[1] * self.dims["unitDist"] + self.dims["buffer"]
        self.go = 0
    
    def processMove(self, time):
        
        #If agent not mapped to current position, agent 
        #has not yet reached its goal, and if its start time has passed, we tell it to move
        if len(self.path) > 1:
            if self.pathsIndex < len(self.path) - 1:
                if self.startTime <= time:
                    self.go = 1
        #initialize agent's change in x,y to 0      
        dx, dy = 0, 0
        if self.go:
            nextNode = self.pixPath[self.pathsIndex + 1]
            if self.px < nextNode[0]:
                 self.px += self.vel
                 dx = 1
            elif self.px > nextNode[0]:
                self.px -= self.vel
                dx = -1
            #avoids moving diagonally
            if not dx:
                if self.py < nextNode[1]:
                    self.py += self.vel
                    dy = 1
                elif self.py > nextNode[1]:
                    self.py -= self.vel
                    dy = -1
            #if we've made it to the nextNode, update position attribute
            if (self.px, self.py) == nextNode:
                self.pathsIndex += 1
                #if nextNode happens to be the goal, tell the agent to stop
                if self.pathsIndex < len(self.path) - 1:
                    
                    self.pos = self.path[self.pathsIndex].coords
                else:
                    self.pos = self.goal
                    self.go = 0
        
        #convert the dx, dy to number of pixels to move
        return [d * self.vel for d in [dx, dy]] 
            

class Graph:
    #need to return a set of edges and vertices
    def __init__(self, xBound, yBound, numAgents, gui):
        
        self.V = [] #set of vertices
        self.xBound = xBound #num x-vals
        self.yBound = yBound #num y-vals
        self.gui = gui
        self.dims = self.gui.dims
        self.timeStep = 0
        self.iterCount = 0
        
        self.xI = [] #set of initial positions
        self.xG = [] #set of goal positions
        self.trgPos = ()
        self.flgV = None
        self.trgRad = None
        
        
        # create the set of vertices
        #organize into row, cols for indexing
        for i in range(self.xBound):
            row = []
            for j in range(self.yBound):
                row.append(Vertex((i, j), (self.xBound, self.yBound)))
            self.V.append(row)
        for row in self.V:
            for v1 in row:
                v1.setAdj(self.V)
        self.V = tuple(self.V)
        
        #currently the entire graph
        aInitBounds = (self.xBound - 1 , self.yBound - 1)
        
        #initialize agents on the graph
        self.agents = []
        for n in range(numAgents):
            uniquePos = 0
            x = None
            y = None
            while not uniquePos:
                x = random.randint(0, aInitBounds[0])
                y = random.randint(0, aInitBounds[1])
                fail = 0
                for i in self.agents:
                    if (x,y) == i.pos:
                        fail = 1
                        break
                if fail:
                    continue
                else:
                    uniquePos = 1
            self.agents.append(Agent((x,y), self.gui.dims))
            self.xI.append(self.V[x][y])
            self.xG.append(self.V[x][y])
            
    def generate_xG(self):
        #called when "Generate" button is presseed
        #     -- identifies new goal nodes and routes agents to these
        
        valid = 1   #make sure we're not switching goals and recalculating in the middle of travel
        for a in self.agents:
            if a.go:
                valid = 0
                break
        if valid:
            self.timeStep = 0
            self.iterCount = 0
            self.trgPos = ()
            self.flgV = None
            
            #goals initialized randomly
            if self.gui.ModeVar.get() == "RAND": 
                
                #update initial positions current agents positions
                #(they're forced to reach goal formation before genearating a new one)
                self.xI = self.xG
                self.xG = []
                for a in self.agents:
                    a.clearAssignments()
                
                #random xG
                for i in range(len(self.agents)):
                    uniquePos = 0
                    x = None
                    y = None
                    while not uniquePos:
                        x = random.randint(0, self.xBound - 1)
                        y = random.randint(0, self.yBound - 1)
                        fail = 0
                        for j in self.xG:
                            if (x,y) == j.coords:
                                fail = 1
                                break
                        if fail: continue
                        else: uniquePos = 1
                    self.xG.append(self.V[x][y])
                    
                #hide any targets from previous rounds
                self.gui.gridcanv.itemconfig(self.gui.trgObjs[0], state = tk.HIDDEN)   

                paths = self.DAG_Paths()
                controlPolicy = scheduledDAG(self, paths) 

            #surround target      
            elif self.gui.ModeVar.get() == "TARG":
                #reassign initial positions within some bounds so as to avoid
                # generating a target such that an agent is already within the required radius of separation
                agentBounds = (self.xBound - 1, (self.yBound - 1)//3)
                self.xI = []
                self.xG = []
                
                self.trgRad = self.gui.radVar.get()
            
                for i in range(len(self.agents)):
                    uniquePos = 0
                    x = None
                    y = None
                    while not uniquePos:
                        x = random.randint(0, agentBounds[0])
                        y = random.randint(0, agentBounds[1])
                        fail = 0
                        for j in self.xI:
                            if (x,y) == j.coords:
                                fail = 1
                                break
                        if fail: continue
                        else: uniquePos = 1
                    self.xI.append(self.V[x][y])
                    self.agents[i].pos = self.V[x][y].coords
                    a = self.agents[i]
                    a.clearAssignments()
                    self.gui.gridcanv.coords(self.gui.agentObjs[i], a.px - self.gui.agentRad, a.py - self.gui.agentRad,
                                           a.px + self.gui.agentRad, a.py + self.gui.agentRad)
      
                #randomly initialize within some bounds
                self.trgPos = (random.randint(0 , self.xBound - 1),
                               random.randint(agentBounds[1] + 3, self.yBound - 1))
                
                trgPix = [c * self.gui.dims["unitDist"] + self.gui.dims["buffer"] for c in self.trgPos]
                self.gui.gridcanv.coords(self.gui.trgObjs[0], trgPix[0] - self.gui.agentRad, trgPix[1] - self.gui.agentRad,
                                           trgPix[0] + self.gui.agentRad, trgPix[1] + self.gui.agentRad)
                self.gui.gridcanv.itemconfig(self.gui.trgObjs[0], state = tk.NORMAL)
                
                flgs, circ = self.flagVs(self.trgRad)
              
                self.getTargXG(circ)
            
                paths = self.DAG_Paths(flgs)
                controlPolicy = scheduledDAG(self, paths)  


    def decrRad(self):
        
        valid = 1   #make sure we're not switching goals and recalculating in the middle of travel
        for a in self.agents:
            if a.go:
                valid = 0
                break
        if valid:
            
            self.timeStep = 0
            self.iterCount = 0
            self.flgV = None
            self.xI = self.xG
            self.xG = []
            self.trgRad -= 1
            for a in self.agents:
                a.clearAssignments()
            
            flgs, circ = self.flagVs(self.trgRad, True)
            self.getTargXG(circ)
            paths = self.DAG_Paths(flgs)
            controlPolicy = scheduledDAG(self, paths)  
         
    def getTargXG(self, circ):
        
        circfnce = len(circ)
        numAssnd = 0
        r = self.trgRad
        circ0 = circ
        
        while circfnce < len(self.agents) - numAssnd:
            print("newiter")
            for i in range(len(circ0)):
                self.xG.append(self.V[circ0[i][0]][circ0[i][1]])
            
            numAssnd += circfnce
            trash, circ0 = self.flagVs(r + 1)
            circfnce = len(circ0)
            r += 1
        
        leftover = (len(self.agents) - numAssnd)
        for i in range(leftover):
            self.xG.append(self.V[circ0[i][0]][circ0[i][1]])
            
    
    def flagVs(self, radius, first = False):
        
        flagList = []
        
        if first:  
            if self.yBound - 1 - self.trgPos[1] <= radius:
                if self.trgPos[0] < radius:
                    x = self.trgPos[0]
                    y = self.trgPos[1]
                    while x > -1:
                        while y < self.yBound - 1:
                            flagList.append((x,y))
                            x -= 1
                            y += 1
    
                elif self.xBound - 1 - self.trgPos[0] <= radius:
                    x = self.trgPos[0]
                    y = self.trgPos[1]
                    while x < self.xBound - 1:
                        while y < self.yBound - 1:
                            flagList.append((x,y))
                            x += 1
                            y += 1
        
        #gets the smallest circle agents can populate
        circ = []
        dy = radius
        while dy > -1:
            dx = radius - dy
            for i in [1, -1]:
                for j in [1, -1]:
                        v = (self.trgPos[0] + dx*i, self.trgPos[1] + dy * j)
                        if 0 <= self.trgPos[0] + dx*i <= self.xBound - 1:
                            if 0 <= self.trgPos[1] + dy*j <= self.yBound - 1:
                                if v not in flagList:
                                    if v not in circ:
                                        circ.append(v)
            dy -= 1
        
        
        #flags everything inside this circle
        dy = radius - 1
        while dy > -1:
            dx = radius - dy - 1
            for i in [1, -1]:
                for j in [1, -1]:
                        v = (self.trgPos[0] + dx*i, self.trgPos[1] + dy * j)
                        if 0 <= self.trgPos[0] + dx*i <= self.xBound - 1:
                            if 0 <= self.trgPos[1] + dy*j <= self.yBound - 1:
                                if v not in flagList:
                                    flagList.append(v)
            dy -= 1

        return [flagList, circ]
    
    def DAG_Paths(self, flgs = None):
        #essentially a Breadth First Search -- alsi records paths and is able to avoid flagged nodes

        costMatrix = np.zeros((len(self.xI), len(self.xG)))
        
        pathsMatrix = [[] for i in range(len(self.xI))]
        for i in range(len(pathsMatrix)):
            pathsMatrix[i] = [None for i in range(len(self.xG))]
        
        for startNode in self.xI:
            visitedMatrix = np.zeros((self.xBound, self.yBound))
            
            parentMatrix = [[] for i in range(self.xBound)]
            
            for i in range(len(parentMatrix)):
                parentMatrix[i] = [None for i in range(self.yBound)]
            visitedMatrix[startNode.coords[0]][startNode.coords[1]] = 1
            
            q = queue.Queue()
            q.put(startNode)
            
            while not q.empty():
                current_node = q.get()
                
                fBool = 0
                if flgs:
                    for f in flgs:
                        if current_node.coords == f:
                            fBool = 1
                            break
                    if fBool:
                        continue
                
                for v in current_node.adj:
                    if not visitedMatrix[v.coords[0]][v.coords[1]]:
                        q.put(v)
                        visitedMatrix[v.coords[0]][v.coords[1]] = 1
                        parentMatrix[v.coords[0]][v.coords[1]] = current_node
                
                if current_node in self.xG:
                    if not costMatrix[self.xI.index(startNode)][self.xG.index(current_node)]:
                        parent = current_node
                        lineage = [parent]    
                        while(parent != startNode):
                            parent = parentMatrix[parent.coords[0]][parent.coords[1]]
                            lineage.append(parent)
                        costMatrix[self.xI.index(startNode)][self.xG.index(current_node)] = len(lineage) - 1
                        pathsMatrix[self.xI.index(startNode)][self.xG.index(current_node)] = lineage
       
        xI_index, xG_index = HungAlg(costMatrix)
        dagPaths = []
       
        for i in range(len(xI_index)):
            path = pathsMatrix[xI_index[i]][xG_index[i]]
           
            path.reverse()
            dagPaths.append(path)
       
        
        return dagPaths
    
    
          
# In[41]:


class Vertex:
    """A general class of vertices. Attributes: coordinates, adjacent vertices. 
    I think this needs to be a subclass of Graph, so that it inherits the x/y bounds"""
    def __init__(self, pos, bounds):
        #pos and bounds are (x,y) tuples
        self.x = pos[0]
        self.y = pos[1]
        self.coords= pos
        self.bounds = bounds
        self.adj = []
    
    def __repr__(self):
        return str(self.coords)
            
    def __eq__(self, other):
        if self.coords == other.coords:
            result = True
        else:
            result = False
        return result
    
    def dist(self, other):
        d = abs(self.coords[0] - other.coords[0]) + abs(self.coords[1] - other.coords[1])
        return d
    
    def setAdj(self, V):
        #The best way I can think of to do this is run through all vertices in V and add to adj list if dist = 1
        #  (and also if the vertex is in the graph)
        #  I wonder if theres a more efficient way to do this
        
        for row in V:
            for v in row:
                if self.dist(v) == 1:
                    if v.coords[0] >= 0 and v.coords[0]<self.bounds[0]:
                        if v.coords[1] >= 0 and v.coords[1] < self.bounds[1]:
                            self.adj.append(v)
                            

                            
class scheduledDAG:
    #initialized with agents, uses their paths to create DAC from start to goals, a reverse to identify standalone goal vertices, 
    #   (represented by dictionaries) and another dict to keep track of what nodes visited each time step
    
    def __init__(self, G, paths):    
        self.ID_Matrix = []
        self.nodeID_Vertex = {}
        self.unprocessedPaths = paths
        self.nodePaths = []
    
        self.initNodes = []
        self.goalNodes = []
        
        self.startTimeAssignment = 0
        self.G = G
        self.DAG = {}
        
        self.agentStartCount = 0
        self.timeDict = {}
        
        
        for row in self.G.V:
            r = []
            for col in row:
                r.append([False, None])  #first element flagged to 1 if visted, second element will record associated ID
            self.ID_Matrix.append(r)
        self.nodeID_count = 0
        for p in range(len(self.unprocessedPaths)):
            pth = self.unprocessedPaths[p]
            nodePath = [] 
            for v in range(len(pth)):  
                thisNode = pth[v]
                assigned = self.ID_Matrix[thisNode.coords[0]][thisNode.coords[1]][0] #Boolean
                nodeID = self.ID_Matrix[thisNode.coords[0]][thisNode.coords[1]][1]  #integer ID
             
                if not assigned: #If the vertex does not have an existing ID:
                    
                    self.ID_Matrix[thisNode.coords[0]][thisNode.coords[1]][0] = True #flag it
                    self.ID_Matrix[thisNode.coords[0]][thisNode.coords[1]][1] = self.nodeID_count  #associate
                    
                    assigned = self.ID_Matrix[thisNode.coords[0]][thisNode.coords[1]][0]
                    nodeID = self.ID_Matrix[thisNode.coords[0]][thisNode.coords[1]][1]
                    
                    self.nodeID_Vertex[nodeID] = thisNode
                    
                    nodePath.append(nodeID)
                    
                    self.timeDict[nodeID] = []  #initialize to track time steps agents are at nodes
                    self.nodeID_count += 1     # increment for next assignment
                    
                else:   #Else it has been flagged
                    nodePath.append(nodeID)
                    
                if v == 0: #v is the first node of a path
                    self.initNodes.append(nodeID)
                    
                if v == len(pth) - 1:   #if v is the last vertex in the path, set agents nodeGoal attribute
                    self.goalNodes.append(nodeID)
            self.nodePaths.append(nodePath)  
 
        standalones = self.constructOrigDAG()  #constructs class attr original DAG, returns list of standalone goal nodes
        
        initNodesScheduled = {}   #used when searching from standalone goal to nearest init node
        for n in self.initNodes:  #keep track of which init nodes to ignore
            initNodesScheduled[n] = False
        
        goalNodesScheduled = {}
        for n in self.goalNodes:
            goalNodesScheduled[n] = False
            
            
        subDAG = self.DAG.copy() #doing this so we still have a copy of original (original is the only one stored in memory)
        
        while any(sched == False for sched in initNodesScheduled.values()):  #while any of the init nodes have not been scheduled
    
            schedDicts = [initNodesScheduled, goalNodesScheduled]
            thisGoal = standalones[0]   #choose a standalone goal node
            
            #This function finds closest init node, records path from init to goal,
            #    and updates the initNodesScheduled variable 
            newNodePairInfo = self.scheduleAgentsBFS(thisGoal, subDAG, schedDicts, G)
            assignedInit = newNodePairInfo[0]
            nodePath = newNodePairInfo[1]
            initNodesScheduled = newNodePairInfo[2][0]
            goalNodesScheduled = newNodePairInfo[2][1]
            
            schedDicts = [initNodesScheduled, goalNodesScheduled]
 
            newInfoDAG = self.updateDAG(subDAG, nodePath)
            subDAG = newInfoDAG[0]
            standalones = newInfoDAG[1]
            
    def updateDAG(self, subDAG, nodePath):
        
        if len(nodePath) > 1:
            for i in range(len(nodePath) - 1): #Discluding the last element; should be standalone goal and not present in DAG
                fromNode = nodePath[i]
                toNode = nodePath[i+1]
                subDAG[fromNode][toNode] -= 1
                if subDAG[fromNode][toNode] == 0:
                    #If count is down to 0, delete the edge between fromNode and toNode
                    subDAG[fromNode] = self.removeSubDictKey(subDAG[fromNode], toNode)
        else: #else its mapped to itself, edge only has one count and you should just delete it
            loopNode = nodePath[0]
            del subDAG[loopNode]
        
        nodesToDelete = []
        for fromNode in subDAG.keys():
            if not subDAG[fromNode]:   #if all outward edges have been deleted, remove key
                nodesToDelete.append(fromNode)
                
        for n in nodesToDelete:
            del subDAG[n]
        
        nodesPresent = []
        for fromNode, toNodeDict in subDAG.items():
            if fromNode not in nodesPresent:
                nodesPresent.append(fromNode)
            for toNode in toNodeDict.keys():
                if toNode not in nodesPresent:
                    nodesPresent.append(toNode)
            
        newStandalones = self.findStandaloneGoals(subDAG, nodesPresent)
        
        return [subDAG, newStandalones]

    def removeSubDictKey(self, subDict, key):
        newSubDict = subDict.copy()
        del newSubDict[key]
        return newSubDict
            
    def constructOrigDAG(self):
       
       directedEdges = []
       loopEdges = []
       allNodesPresent = []      
       for path in self.nodePaths:
           for v in path:   #use this to check keys in DAG for standalone goal nodes
               if v not in allNodesPresent:
                   allNodesPresent.append(v)
           if len(path) == 1:
               loopEdges.append(path[0])
           else:
               for v in range(len(path) - 1):   #want to disclude goal vertex -- won't be a key in forward DAC and will only be tail end of an edge
                    thisNode = path[v]
                    nextNode = path[v+1]
                    directedEdges.append((thisNode, nextNode))  #append them all
                    
              
       #now use this edge set to construct forward/backward DACs
       for edge in directedEdges:
            fromNode = edge[0]
            toNode = edge[1]
            if fromNode not in self.DAG.keys():
                self.DAG[fromNode] = {} #initialize to empty list
                self.DAG[fromNode][toNode] = 1  #fill in DAG; need to keep track of number occurrences of each edge
            else: #fromNode is in the keys; need to check whether toNode is in its list or not
                if toNode in self.DAG[fromNode].keys():
                    #if it is, increment the count
                    self.DAG[fromNode][toNode] += 1
                else:
                    self.DAG[fromNode][toNode] = 1 #else add it with a count of 1
                    
            
       for loop in loopEdges:  #for nodes that are mapped to themselves
           boolBreakLoop = 0
           for key in self.DAG.keys():
               if loop in self.DAG[key].keys():
                   boolBreakLoop = 1
                   break
           if boolBreakLoop:  #go to next loopEdge
               continue
           else:
               self.DAG[loop] = {}
               self.DAG[loop][loop] = 1
           
       standaloneGoals = self.findStandaloneGoals(self.DAG, allNodesPresent)
       return standaloneGoals
   
      
    def findStandaloneGoals(self, dag, nodesTravelled):
       #will return a list of standalone goal nodes
       standalones = []       #if the nodes have been travelled but not in the keys, then agents who visited these nodes did not move from them

       for i in nodesTravelled: 
            if i not in dag.keys():
                if i in self.goalNodes: #Honestly I think this line isn't necessary
                    standalones.append(i)
            else: #else i is in the keys
                if dag[i] == {i:1}: #i is mapped to itself
                    standalones.append(i)
                    
       return standalones 
   
    
    def scheduleAgentsBFS(self, goal, dag, schedDicts, G):
        #should be noted that this function actually initializes agent objects
    
        initNodesScheduled = schedDicts[0]
        goalNodesScheduled = schedDicts[1]
        
        goalNodesScheduled[goal] = True
    
        visitedNodes = {}
        
        parentDict = {}
            
        q = queue.Queue()
        q.put(goal)
        visitedNodes[goal] = True
    
        while not q.empty():
            current = q.get()
            
            #put this condition first so if current is an unsched init node, we don't go through searching for its parents and creating a key for 
            #   it in parent dict, etc
            if current in initNodesScheduled.keys():
                if not initNodesScheduled[current]:  #we've encountered an initial node that has not been assigned yet
                    initNodesScheduled[current] = True
                    break #break the while loop 
            
            if current not in parentDict.keys():
                parentDict[current] = []
            for key, val in dag.items():  #look through tail ends of directed edges for the current node
                for outNode in val.keys():           #add the key(s) to the queue, mark as visited, add key(s) to current's value in parentDict
                    if current == outNode:
                        if key not in visitedNodes.keys():
                            visitedNodes[key] = True
                            q.put(key)
                        parentDict[current].append(key)
        
        #Construct agents path - using nodeID's at first, but translate to vertices so it can be used for movement in the GUI
        assignedInit = current 
        newNodePath = [current]  #this is the initial node
        vertexPath = [self.nodeID_Vertex[current]]   #again, initial vertex
        
        while current != goal:  
            for key, val in parentDict.items():
                for v in val:
                    if v == current:    #I feel like this could be problematic, because what if current is parent to more than one?
                        current = key                  
                        newNodePath.append(current)
                        vertexPath.append(self.nodeID_Vertex[current])
       
        #determine start time
        ts = []
        for i in range(len(newNodePath)):
            t = self.agentStartCount + i
            ts.append(t)
            
        brkBool = 0
        for i in range(len(newNodePath)):
            node = newNodePath[i]
            if ts[i] in self.timeDict[node]:
                self.agentStartCount += 1
                
                brkBool = 1
                break
        
        if brkBool:
           ts = [t + 1 for t in ts]
           
        for i in range(len(newNodePath)):
           self.timeDict[newNodePath[i]].append(ts[i])
        agent= None

        for i in range(len(G.agents)):
            a = G.agents[i]
            if a.pos == self.nodeID_Vertex[newNodePath[0]].coords:
                agent = a
                break
        startTime = self.agentStartCount
        agent.mapPixelPath(vertexPath, startTime)
        schedDicts = [initNodesScheduled, goalNodesScheduled ]
                    
        return [assignedInit, newNodePath, schedDicts]
    
    