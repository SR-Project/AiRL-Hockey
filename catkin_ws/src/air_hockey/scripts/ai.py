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
	goal_px, goal_py = 0.565, 0.050

	reachable = puck_py <= 1.025 # half court

	x, y = 0, 0
	if not reachable:
		target_px, target_py = (0.565, 0.150) # defense position above goal

		def defend_goal(goal, p):
			diff = goal - p
			if abs(diff) < 0.05: return  0
			elif diff > 0:    return -1 # negate since if the distance on y is > i need to go down
			else:             return 1
		x = defend_goal(target_px, px)
		y = defend_goal(target_py, py)

	else:
		print("reachable")
		if puck_vy <= 0:
			if puck_px < px: x = 1
			if puck_px > px: x = -1
			if puck_py < py: y = 1
			if puck_py > py: y = -1

		else:
			too_fast = np.linalg.norm((puck_dx, puck_dy)) > 0.1 # TODO: check
			if too_fast:
				def save_goal(goal, p):
					diff = goal - p
					if abs(diff) < 0.01: return  0
					elif diff > 0:    return  -1
					else:             return 1
				x = save_goal(goal_px, px)
				y = save_goal(goal_py, py)

			else:
				if puck_px < px: x = 1
				if puck_px > px: x = -1
				if puck_py < py: y = 1 
				if puck_py > py: y = -1

	print(f"pre-conversion: {(x,y)}")
	return x, y