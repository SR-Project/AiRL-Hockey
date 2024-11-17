import numpy as np


def act(goal, p, th):
	diff = goal - p
	if abs(diff) < 0.05: return  0
	elif diff > 0:    return 1 
	else:             return -1


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
	goal_px, goal_py = 0.565, 0.050

	reachable = puck_py <= 0.95 # half court reachable

	x, y = 0, 0
	if not reachable:
		#target_px, target_py = (0.565, 0.100) # defense position above goal
		target_px, target_py = (0.565, 0.200)

		x = act(target_px, px, 0.05)
		y = act(target_py, py, 0.05)

	else:
		if puck_vy <= 0:
			x = act(puck_px, px, 0.01)
			y = act(puck_py, py, 0.01)
			
		else:
			too_fast = np.linalg.norm((puck_dx, puck_dy)) > 0.1 # TODO: check
			if too_fast:
				x = act(goal_px, px, 0.01)
				y = act(goal_py, py, 0.01)

			else:
				x = act(puck_px, px, 0.01)
				y = act(puck_py, py, 0.01)

	return -x, -y # negate since the coordinates go in opposite directions w.r.t. image