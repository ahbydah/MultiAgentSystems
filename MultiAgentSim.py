# -*- coding: utf-8 -*-
"""
Created on Sat Nov 10 01:37:32 2018

@author: Aubrey
"""


import numpy as np
import random

from GraphControlClasses import *
import tkinter as tk


class MultiAgentSimulation():
    def __init__(self, master):
        
        #####################################################
        ##
        ##   INSTANCE VARIABLE INITIALIZATION
        ## **************************************
        ## - sets values for GUI dimensions and colors 
        ## - initializes Graph obj G, which in turn creates 
        ##   agents at random start positions
        ## - values for agent "velocity" ie how many pixels to 
        ##   move each mainloop iteration
        ##
        ###################################################
        
        self.dims = {}
        self.dims["unitDist"] = 30
        self.dims["buffer"] = self.dims["unitDist"] // 2
        
        numRows = 15
        numCols = 15
        numAgents = 30
        self.G = Graph(numRows, numCols, numAgents, self)
        
        self.agentVel = self.dims["unitDist"]//5
        self.agentRad = self.dims["unitDist"]//3
        
        self.dims["gW"] = (self.G.xBound - 1) * self.dims["unitDist"] 
        self.dims["gH"] = (self.G.yBound - 1) * self.dims["unitDist"] 
        self.dims["tlbrH"] = self.dims["gH"]
        self.dims["tlbrW"] = self.dims["gW"] // 3
        self.dims["tlbrX"] = self.dims["gW"] + 2 * self.dims["buffer"]
        self.dims["tlbrY"] = self.dims["buffer"]
        self.dims["dispW"] = self.dims["gW"] + self.dims["tlbrW"] + 3 * self.dims["buffer"]
        self.dims["dispH"] = self.dims["gH"] + 2 * self.dims["buffer"]
        self.dims["buttonX"] = self.dims["tlbrX"] + self.dims["tlbrW"] // 8
        self.dims["buttonW"] = 3 * (self.dims["tlbrW"] // 4)
        self.dims["buttonH"] = 2 * (self.dims["buttonW"] // 3)
        
        self.colors = {"bg" :"#e6e6e6",  
                  "gridLines": "#c8c8c8", 
                  "agents" : "#104c7c", 
                  "toolbar": "#c4c3c9",
                  "buttonBg": "#e0e5e3" ,
                  "buttonTxt": "#3a5148" ,
                  "targ": "#ff0400" ,
                  "perims": "#44335b"}
        #####################################################
        ##
        ##   GUI CONFIGURATION
        ## **************************************
        ## - creates master frame, canvas, and toolbar frame, and objects that populate these
        ## 
        ##
        ##
        ###################################################
       
        self.master = master
        self.master.configure(bg = self.colors["bg"])
        
        
        self.gridcanv = tk.Canvas(self.master,  width = self.dims["gW"] + 2*self.dims["buffer"],
                             height = self.dims["gH"]+ 2*self.dims["buffer"],
                             bg = self.colors["bg"])
        
        self.tlbr = tk.Frame(self.master, width = self.dims["tlbrW"]+ 2*self.dims["buffer"], 
                        height = self.dims["tlbrH"]+ 2*self.dims["buffer"],
                        bg = self.colors["toolbar"])
        
        self.gridcanv.pack(side = tk.LEFT, padx = self.dims["buffer"], pady = self.dims["buffer"] )
        self.tlbr.pack(side = tk.LEFT, padx = self.dims["buffer"], pady = self.dims["buffer"])
        self.drawGrid()
        self.trgObjs = self.initTargs()
        self.agentObjs = self.initAgents()
        self.goalObjs = self.init_xG()
        self.widgObjs = self.fillToolbar()

        
        #####################################################
        ##
        ##       MAIN ANIMATION LOOP
        ## *************************************
        ## 
        ##
        ###################################################
        
        self.master.after(0, self.animation)
    def animation(self):
        
        self.G.goalMode = self.ModeVar.get()

        for i in range(len(self.G.xG)):
            pos = self.G.xG[i]
            x = pos.coords[0]
            y = pos.coords[1]
            px = x * self.dims["unitDist"] + self.dims["buffer"]
            py = y * self.dims["unitDist"] + self.dims["buffer"]
            self.gridcanv.coords(
                    self.goalObjs[i], 
                    px - self.agentRad - 1, py - self.agentRad - 1,
                    px + self.agentRad + 1, py + self.agentRad +1) 
        
        for i in range(len(self.agentObjs)):
        
            agent = self.G.agents[i]
            dx, dy = agent.processMove(self.G.timeStep)
            self.gridcanv.move(self.agentObjs[i], dx, dy)
      
        self.master.after(75, self.animation )
        self.G.iterCount += 1
        if self.G.iterCount % 5 == 0:
            self.G.timeStep += 1    
            
    def drawGrid(self):
        x = self.dims["buffer"]
        y = self.dims["buffer"]
        for c in range(self.G.xBound):
            self.gridcanv.create_line(x, y, x, y + self.dims["gH"], 
                             fill = self.colors["gridLines"], width = 2)
            x += self.dims["unitDist"]
        x = self.dims["buffer"]    
        for r in range(self.G.yBound):
            self.gridcanv.create_line(x, y, x+self.dims["gW"], y,
                             fill = self.colors["gridLines"], width = 2)
            y += self.dims["unitDist"]
            
    def init_xG(self):
        goalObjs = []
        for pos in self.G.xG:
            x = pos.coords[0]
            y = pos.coords[1]
            px = x * self.dims["unitDist"] + self.dims["buffer"]
            py = y * self.dims["unitDist"] + self.dims["buffer"]
            obj = self.gridcanv.create_oval(px - self.agentRad, py - self.agentRad,
                                     px + self.agentRad, py + self.agentRad)
            goalObjs.append(obj)
        return goalObjs
  
    def fillToolbar(self):
        objs = []
        
        self.ModeVar = tk.StringVar()
        self.rand_xG = tk.Radiobutton(self.tlbr, text = "Random",var = self.ModeVar, value = "RAND",
                                      font = "Helvetica 12", fg = self.colors["buttonTxt"], 
                                      bg = self.colors["toolbar"])
        self.targ_xG = tk.Radiobutton(self.tlbr, text = "Surround", var = self.ModeVar,
                                      value = "TARG", font = "Helvetica 12", 
                                      fg = self.colors["buttonTxt"], bg = self.colors["toolbar"])  
        
        self.radVar = tk.IntVar()
        self.radOpts = [r+1 for r in range(5)]
        self.radMenu = tk.OptionMenu(self.tlbr, self.radVar, *self.radOpts)
        self.radVar.set(self.radOpts[2])
        self.decr = tk.Button(self.tlbr, text = "Decrement \n Radius",
                                 fg = self.colors["buttonTxt"], bg = self.colors["buttonBg"],
                                 command = self.G.decrRad, font = "Helvetica 12")
        
        self.generate = tk.Button(self.tlbr, text = "Generate", 
                             fg = self.colors["buttonTxt"], bg = self.colors["buttonBg"], 
                             command = self.G.generate_xG, font = "Helvetica 19 bold")
        self.rand_xG.grid(row = 0, column = 0, padx = 5, pady = 10)
        self.targ_xG.grid(row = 0, column = 1, padx = 5, pady = 10)
        self.radMenu.grid(row = 1, column = 0, padx = 5, pady = 10)
        self.decr.grid(row = 1, column = 1, padx = 5, pady = 10)
        self.generate.grid(row = 2, columnspan = 2, padx = 5, pady = 10)
        
        objs.append(self.radMenu)
        objs.append(self.decr)
        objs.append(self.generate)
        objs.append(self.rand_xG)
        objs.append(self.targ_xG)
    
        return objs
    
    def initAgents(self):
        aObjs = []
        for a in self.G.agents:
            obj = self.gridcanv.create_oval(a.px - self.agentRad, a.py - self.agentRad,
                             a.px + self.agentRad, a.py + self.agentRad, 
                             fill = self.colors["agents"])
            aObjs.append(obj)
        return aObjs
    
    def initTargs(self):
        
        trgs = [] #setting it up for the future -- intend to have option for multiple targs
        trgPix = [self.dims["buffer"], self.dims["buffer"]]
        t1 = self.gridcanv.create_oval(trgPix[0] - self.agentRad, trgPix[1] - self.agentRad,
                                                       trgPix[0] + self.agentRad, trgPix[1] + self.agentRad,
                                                       fill = self.colors["targ"], state = tk.HIDDEN)
        trgs.append(t1)
        return trgs
    
    
        
def main(): 
    root = tk.Tk()
    app = MultiAgentSimulation(root)
    root.mainloop()

if __name__ == '__main__':
    main()
