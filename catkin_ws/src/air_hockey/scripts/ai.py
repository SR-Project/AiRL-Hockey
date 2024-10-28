import numpy as np



def move(puck_x, puck_y, puck_dx, puck_dy, mallet_x, mallet_y, mallet_dx, mallet_dy):

    '''px, py = mallet_y, mallet_x
    vx, vy = mallet_dy, mallet_dx

    puck_px, puck_py = puck_y, puck_x
    puck_vx, puck_vy = puck_dy, puck_dx'''

    px, py = mallet_x, mallet_y
    vx, vy = mallet_dx, mallet_dy

    puck_px, puck_py = puck_x, puck_y
    puck_vx, puck_vy = puck_dx, puck_dy

    # Coordinate della proprio porta
    goal_px, goal_py = -1.046886, 0.000117


    reachable = puck_px <= 0

    x, y = 0, 0
    if not reachable:

        #print("Not reachable")

        # Coordinate con cui difendersi
        target_px, target_py = (-0.912345, 0.000117)

        def defend_goal(goal, p):

            #print(goal, p)
            diff = goal - p
            #print(diff)
            if abs(diff) < 0.05: return  0
            elif diff < 0:    return  -1
            else:             return 1
    
    

        x = defend_goal(target_px, px)
        y = defend_goal(target_py, py)
        # print('{:15} {:4d} {:4d}'.format('not reachable', x, y))

    else:
        #print("reachable")
        if puck_vy <= 0:
            #print("Puck still")
            #print(puck_px, px)
            #print(puck_py, py)
            if puck_px < px: x = -1
            if puck_px > px: x = 1
            if puck_py > py: y = 1
            if puck_py < py: y = -1
#                print('{:15} {:4d} {:4d}'.format('stationary', x, y))
        else:
            too_fast = np.linalg.norm([puck_vx, puck_vy]) > 0.8*0.015
            if too_fast:
                #print("Puck too fast")
                def save_goal(goal, p):
                    diff = goal - p
                    if abs(diff) < 0.02: return  0
                    elif diff > 0:    return  1
                    else:             return -1
                x = save_goal(goal_px, px)
                y = save_goal(goal_py, py)
#                    print('{:15} {:4d} {:4d}'.format('too fast', x, y))
            else:
                #print("Puck vel ok")
                if puck_px < px: x = -1
                if puck_px > px: x = 1
                if puck_py < py: y = -1
                if puck_py > py: y = 1
#                    print('{:15} {:4d} {:4d}'.format('slow', x, y))
    print(x,y)
    return x, y


