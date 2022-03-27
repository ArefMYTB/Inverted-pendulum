# -*- coding: utf-8 -*-

# python imports
from math import degrees

# pyfuzzy imports
from fuzzy.storage.fcl.Reader import Reader

import numpy as np

f = 0

# fuzzyValues: [left, mid, right, value]
# PA
paValue = {
    "up_more_right": [0, 30, 60, 0.],
    "up_right": [30, 60, 90, 0.],
    "up": [60, 90, 120, 0.],
    "up_left": [90, 120, 150, 0.],
    "up_more_left": [120, 150, 180, 0.],
    "down_more_left": [180, 210, 240, 0.],
    "down_left": [210, 240, 270, 0.],
    "down": [240, 270, 300, 0.],
    "down_right": [270, 300, 330, 0.],
    "down_more_right": [300, 330, 360, 0.]
}
# PV
pvValue = {
    "cw_fast": [-210, -200, -100, 0.],
    "cw_slow": [-200, -100, 0, 0.],
    "stop": [-100, 0, 100, 0.],
    "ccw_slow": [0, 100, 200, 0.],
    "ccw_fast": [100, 200, 210, 0.]
}
# CP
cpValue = {
    "left_far": [-12, -10, -5, 0.],
    "left_near": [-10, -2.5, 0, 0.],
    "stop": [-2.5, 0, 2.5, 0.],
    "right_near": [0, 2.5, 10, 0.],
    "right_far": [5, 10, 12, 0.]
}
# CV
cvValue = {
    "left_fast": [-7, -5, -2.5, 0.],
    "left_slow": [-5, -1, 0, 0.],
    "stop": [-1, 0, 1, 0],
    "right_slow": [0, 1, 5, 0.],
    "right_fast": [2.5, 5, 7, 0.]
}
# FORCE
forceValue = {
    "left_fast": [-100, -80, -60, 0.],
    "left_slow": [-80, -60, 0, 0.],
    "stop": [-60, 0, 60, 0],
    "right_slow": [0, 60, 80, 0.],
    "right_fast": [60, 80, 100, 0.]
}

class FuzzyController:

    def __init__(self, fcl_path):
        self.system = Reader().load_from_file(fcl_path)

    def _make_input(self, world):
        return dict(
            cp = world.x,                   # Cart position
            cv = world.v,                   # Cart velocity 
            pa = degrees(world.theta),      # Pendulum angle
            pv = degrees(world.omega)       # Pendulum angular velocity
        )

    def _make_output(self):
        return dict(
            force = 0.
        )

    # self.system.calculate(self._make_input(world), output)
    def decide(self, world):
        input = self._make_input(world)
        fuzzification(input)
        inference()
        output = defuzzification()
        backToZero()
        return output['force']

# fuzzification
def fuzzification(inputDict):
    pa(inputDict["pa"])
    pv(inputDict["pv"])
    cp(inputDict["cp"])
    cv(inputDict["cv"])

def pa(value):
    for x, y in paValue.items():
        if(y[0] <= value <= y[1]):
            paValue[x][3] = abs((value - y[0])/(y[1]-y[0]))
        if(y[1] <= value <= y[2]):
            paValue[x][3] = abs((value - y[2])/(y[1]-y[2]))

def pv(value):
    for x, y in pvValue.items():
        if(y[0] <= value <= y[1]):
            pvValue[x][3] = abs((value - y[0])/(y[1]-y[0]))
        if(y[1] <= value <= y[2]):
            pvValue[x][3] = abs((value - y[2])/(y[1]-y[2]))
        elif(value > 200):
            pvValue["ccw_fast"][3] = 1
        elif(value < -200):
            pvValue["cw_fast"][3] = 1
            
def cp(value):
    for x, y in cpValue.items():
        if(y[0] <= value <= y[1]):
            cpValue[x][3] = abs((value - y[0])/(y[1]-y[0]))
        if(y[1] <= value <= y[2]):
            cpValue[x][3] = abs((value - y[2])/(y[1]-y[2]))

def cv(value):
    for x, y in cvValue.items():
        if(y[0] <= value <= y[1]):
            cvValue[x][3] = abs((value - y[0])/(y[1]-y[0]))
        if(y[1] <= value <= y[2]):
            cvValue[x][3] = abs((value - y[2])/(y[1]-y[2]))

def backToZero():
    for item in cvValue:
        cvValue[item][3] = 0
    for item in cpValue:
        cpValue[item][3] = 0
    for item in pvValue:
        pvValue[item][3] = 0
    for item in paValue:
        paValue[item][3] = 0
    for item in forceValue:
        forceValue[item][3] = 0

def inference():
    # Rule 0
    forceValue["stop"][3] = max(forceValue["stop"][3], max(min(paValue["up"][3], pvValue["stop"][3]), 
                                min(paValue["up_right"][3], pvValue["ccw_slow"][3]), 
                                min(paValue["up_left"][3], pvValue["cw_slow"][3])))
    # Rule 1
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_more_right"][3], pvValue["ccw_slow"][3]))
    # Rule 2
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_more_right"][3], pvValue["cw_slow"][3]))
    # Rule 3
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up_more_left"][3], pvValue["cw_slow"][3]))
    # Rule 4
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up_more_left"][3], pvValue["ccw_slow"][3]))
    # Rule 5
    forceValue["left_slow"][3] = max(forceValue["left_slow"][3], min(paValue["up_more_right"][3], pvValue["ccw_fast"][3]))
    # Rule 6
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_more_right"][3], pvValue["cw_fast"][3]))
    # Rule 7
    forceValue["right_slow"][3] = max(forceValue["right_slow"][3], min(paValue["up_more_left"][3], pvValue["cw_fast"][3]))
    # Rule 8
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up_more_left"][3], pvValue["ccw_fast"][3]))
    # Rule 9
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["down_more_right"][3], pvValue["ccw_slow"][3]))
    # Rule 10
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_more_right"][3], pvValue["cw_slow"][3]))
    # Rule 11
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["down_more_left"][3], pvValue["cw_slow"][3]))
    # Rule 12
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_more_left"][3], pvValue["ccw_slow"][3]))
    # Rule 13
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_more_right"][3], pvValue["ccw_fast"][3]))
    # Rule 14
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_more_right"][3], pvValue["cw_fast"][3]))
    # Rule 15
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_more_left"][3], pvValue["cw_fast"][3]))
    # Rule 16
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_more_left"][3], pvValue["ccw_fast"][3]))
    # Rule 17
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["down_right"][3], pvValue["ccw_slow"][3]))
    # Rule 18
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["down_right"][3], pvValue["cw_slow"][3]))
    # Rule 19
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["down_left"][3], pvValue["cw_slow"][3]))
    # Rule 20
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["down_left"][3], pvValue["ccw_slow"][3]))
    # Rule 21
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_right"][3], pvValue["ccw_fast"][3]))
    # Rule 22
    forceValue["right_slow"][3] = max(forceValue["right_slow"][3], min(paValue["down_right"][3], pvValue["cw_fast"][3]))
    # Rule 23
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down_left"][3], pvValue["cw_fast"][3]))
    # Rule 24
    forceValue["left_slow"][3] = max(forceValue["left_slow"][3], min(paValue["down_left"][3], pvValue["ccw_fast"][3]))
    # Rule 25
    forceValue["right_slow"][3] = max(forceValue["right_slow"][3], min(paValue["up_right"][3], pvValue["ccw_slow"][3]))
    # Rule 26
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_right"][3], pvValue["cw_slow"][3]))
    # Rule 27
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_right"][3], pvValue["stop"][3]))
    # Rule 28
    forceValue["left_slow"][3] = max(forceValue["left_slow"][3], min(paValue["up_left"][3], pvValue["cw_slow"][3]))
    # Rule 29
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up_left"][3], pvValue["ccw_slow"][3]))
    # Rule 30
    forceValue["left_fast"][3] =max(forceValue["left_fast"][3],  min(paValue["up_left"][3], pvValue["stop"][3]))
    # Rule 31
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up_right"][3], pvValue["ccw_fast"][3]))
    # Rule 32
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_right"][3], pvValue["cw_fast"][3]))
    # Rule 33
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up_left"][3], pvValue["cw_fast"][3]))
    # Rule 34
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up_left"][3], pvValue["ccw_fast"][3]))
    # Rule 35
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["down"][3], pvValue["stop"][3]))
    # Rule 36
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down"][3], pvValue["cw_fast"][3]))
    # Rule 37
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["down"][3], pvValue["ccw_fast"][3]))
    # Rule 38
    forceValue["left_slow"][3] = max(forceValue["left_slow"][3], min(paValue["up"][3], pvValue["ccw_slow"][3]))
    # Rule 39
    forceValue["left_fast"][3] = max(forceValue["left_fast"][3], min(paValue["up"][3], pvValue["ccw_fast"][3]))
    # Rule 40
    forceValue["right_slow"][3] = max(forceValue["right_slow"][3], min(paValue["up"][3], pvValue["cw_slow"][3]))
    # Rule 41
    forceValue["right_fast"][3] = max(forceValue["right_fast"][3], min(paValue["up"][3], pvValue["cw_fast"][3]))
    # Rule 42
    forceValue["stop"][3] = max(forceValue["stop"][3], min(paValue["up"][3], pvValue["stop"][3]))

def defuzzification():
    global f
    x_ax = np.arange(-100,100,0.1)
    y_ax = [0]*2000

    for i in range(len(x_ax)):
        for x, y in forceValue.items():
            if(y[0] <= x_ax[i] <= y[2]):
                if(y[0] <= x_ax[i] <= y[1]):                # if it is in left half
                    term1 = (x_ax[i]-y[0])/(y[1]-y[0])
                    if(term1 < y[3]):
                        y_ax[i] = max(y_ax[i], term1)
                    else:
                        y_ax[i] = max(y_ax[i], y[3])
                else:                                       # if it is in right half
                    term2 = (x_ax[i]-y[2])/(y[1]-y[2])
                    if(term2 < y[3]):
                        y_ax[i] = max(y_ax[i], term2)
                    else:
                        y_ax[i] = max(y_ax[i], y[3])

    f1 = 0
    f2 = 0
    for i in range(len(x_ax)):
        f1 += y_ax[i] * x_ax[i]
        f2 += y_ax[i]

    if(f == 0):
        ft = 0
    else:
        ft = f1/f2 
    f+=1
    
    return dict(
        force = ft
    )
