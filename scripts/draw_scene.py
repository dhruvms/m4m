import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

import os
import collections

FIG, AX = plt.subplots(figsize=(13, 13))
FRIDGE = True

def ParseFile(filepath):
	objs = {}
	trajs = {}
	with open(filepath, 'r') as f:
		done = False
		while not done:
			line = f.readline()[:-1]

			if len(line) == 0:
				done = True

			if line == 'O':
				line = f.readline()[:-1]
				num_objs = int(line)
				for i in range(num_objs):
					line = f.readline()[:-1]
					obj = [float(val) for val in line.split(',')[:-1]]
					# line = f.readline()[:-1]
					# objs[obj[0]] += [line == 'True']
					objs[obj[0]] = obj[1:]
					objs[obj[0]] += [line.split(',')[-1] == 'True']

				objs = collections.OrderedDict(sorted(objs.items()))

			if line == 'T':
				line = f.readline()[:-1]
				num_moving = int(line)
				for i in range(num_moving):
					line = f.readline()[:-1]
					obj_id = int(line)

					line = f.readline()[:-1]
					num_wps = int(line)

					traj = []
					for i in range(num_wps):
						line = f.readline()[:-1]
						wp = [float(val) for val in line.split(',')]
						traj.append(wp)
					trajs[obj_id] = traj

	return objs, trajs

def DrawScene(filepath, objs, trajs, alpha=1.0):
	codes = [
		Path.MOVETO,
		Path.LINETO,
		Path.LINETO,
		Path.LINETO,
		Path.CLOSEPOLY,
	]

	base = objs[1]
	base_bl = np.array(base[2:4])

	base_extents = np.array(base[8:10])
	base_bl = base_bl - base_extents

	base_rect = patches.Rectangle(
						(base_bl[0], base_bl[1]),
						2 * base_extents[0], 2 * base_extents[1],
						linewidth=2, edgecolor='k', facecolor='none',
						alpha=alpha, zorder=1)
	AX.add_artist(base_rect)

	for obj_id in objs:
		if (FRIDGE and obj_id <= 5):
			continue
		if (not FRIDGE and obj_id <= 1):
			continue

		obj = objs[obj_id]

		obj_shape = obj[0]
		obj_movable = obj[-1]
		ec = 'b' if obj_movable else 'r'
		fc = 'b' if obj_movable else 'r'
		tc = 'w'
		if obj_id >= 100 and obj_id <= 102:
			ec = 'gray'
			fc = 'gray'
			tc = 'k'
		if obj_id == 999:
			ec = 'gold'
			fc = 'gold'
			tc = 'k'

		obj_cent = np.array(obj[2:4])

		if (obj_shape == 0): # rectangle
			obj_extents = np.array(obj[8:10])
			obj_pts = np.vstack([	obj_cent - (obj_extents * [1, 1]),
									obj_cent - (obj_extents * [-1, 1]),
									obj_cent - (obj_extents * [-1, -1]),
									obj_cent - (obj_extents * [1, -1]),
									obj_cent - (obj_extents * [1, 1])]) # axis-aligned

			R = np.array([
					[np.cos(obj[7]), -np.sin(obj[7])],
					[np.sin(obj[7]), np.cos(obj[7])]]) # rotation matrix
			obj_pts = obj_pts - obj_cent # axis-aligned, at origin
			obj_pts = np.dot(obj_pts, R.T) # rotate at origin
			obj_pts = obj_pts + obj_cent # translate back

			path = Path(obj_pts, codes)
			obj_rect = patches.PathPatch(path, ec=ec, fc=fc, lw=1,
								alpha=alpha, zorder=2)

			AX.add_artist(obj_rect)

		elif (obj_shape == 2): # circle
			assert obj[8] == obj[9]

			obj_rad = obj[8]
			obj_circ = patches.Circle(
						obj_cent, radius=obj_rad,
						ec=ec, fc=fc, lw=1,
						alpha=alpha, zorder=2)

			AX.add_artist(obj_circ)

		AX.text(obj_cent[0], obj_cent[1], str(int(obj_id)), color=tc, zorder=3)

	if (trajs):
		for key in trajs:
			if key == 99:
				lc = 'lightgray'
			elif key == 999:
				lc = 'orange'
			else:
				lc = 'cyan'

			traj = np.asarray(trajs[key])
			AX.plot(traj[:, 0], traj[:, 1], c=lc, alpha=0.75, zorder=10)

	AX.axis('equal')
	AX.set_xlim([0.0, 1.2])
	AX.set_ylim([-1.1, 0.0])
	# plt.show()
	plt.savefig(filepath.replace('txt', 'png'), bbox_inches='tight')
	plt.cla()

def main():
	datafolder = os.path.dirname(os.path.abspath(__file__)) + '/../dat/txt/'
	for (dirpath, dirnames, filenames) in os.walk(datafolder):
		for filename in filenames:
			if '.txt' not in filename:
				continue

			# if int(filename.split('.')[0]) % 10 != 0:
			# 	continue

			filepath = os.path.join(dirpath, filename)
			objs, trajs = ParseFile(filepath)
			DrawScene(filepath, objs, trajs)

if __name__ == '__main__':
	main()
