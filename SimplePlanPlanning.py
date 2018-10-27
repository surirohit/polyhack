import numpy as np 

def path_setting(cx,cy,dx,dy):
    # From zero to some delivery position
    if (cx==2.2 and cy==1.6):
        if (dx==2.6 and dy ==0.6):
            return 2
    if (cx==2.2 and cy==1.6):
        if (dx==3.4 and dy ==1.4):
            return 3
    if (cx==2.2 and cy==1.6):
        if (dx==2.4 and dy ==3.4):
            return 4
    if (cx==2.2 and cy==1.6):
        if (dx==0.6 and dy ==2.2):
            return 5
    if (cx==2.2 and cy==1.6):
        if (dx==1.4 and dy ==3.2):
            return 6
    if (cx==2.2 and cy==1.6):
        if (dx==1 and dy ==3.2):
            return 7
    if (cx==2.2 and cy==1.6):
        if (dx==3.6 and dy ==0.6):
            return 8
    if (cx==2.2 and cy==1.6):
        if (dx==3.2 and dy ==3.2):
            return 9

    #from delivery position to zero
    if (cx==2.6 and cy==0.6):
        if (dx==2.2 and dy ==1.6):
            return 20
    if (cx==3.4 and cy==1.4):
        if (dx==2.2 and dy ==1.6):
            return 30
    if (cx==2.4 and cy==3.4):
        if (dx==2.2 and dy ==1.6):
            return 40
    if (cx==0.6 and cy==2.2):
        if (dx==2.2 and dy ==1.6):
            return 50
    if (cx==1.4 and cy==3.2):
        if (dx==2.2 and dy ==1.6):
            return 60
    if (cx==1 and cy==1.6):
        if (dx==2.2 and dy ==1.6):
            return 70
    if (cx==3.6 and cy==0.6):
        if (dx==2.2 and dy ==1.6):
            return 80
    if (cx==3.2 and cy==3.2):
        if (dx==2.2 and dy ==1.6):
            return 90
    return 0
    
def path_planning_simple(path):

    #From zero to goal point
    if path == 2:
        return [[2.6,0.6]]
    if path == 3:
        return [[3.4,1.4]]
    if path == 4:
        return [[2.6,2.6],[2.4,3.4]]
    if path == 5:
        return [[0.6,2.2]]
    if path == 6:
        return [[1.4,3.2]]
    if path == 7:
        return [[1,1.6]]
    if path == 8:
        return [[3.4,1.4],[3.6,0.6]]
    if path == 9:
        return [[2.6,2.6],[3.2,3.2]]

#From goal point to zero
    if path == 20:
        return [[2.2,1.6]]
    if path == 30:
        return [[2.2,1.6]]
    if path == 40:
        return [[2.6,2.6],[2.2,1.6]]
    if path == 50:
        return [[2.2,1.6]]
    if path == 60:
        return [[1.4,3.2]]
    if path == 70:
        return [[2.2,1.6]]
    if path == 80:
        return [[3.4,1.4],[2.2,1.6]]
    if path == 90:
        return [[2.6,2.6],[2.2,1.6]]

        
