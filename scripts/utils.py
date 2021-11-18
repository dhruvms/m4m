import numpy as np
import pybullet as p
from collections import namedtuple

PR2_URDF = '../config/urdf/pr2_description/pr2.urdf'
CLEARANCE = 0.0
GUI = False
CLIENT = 0
TABLE_COLOUR = [0.5, 0.5, 0.5]
STATIC_COLOUR = [0.698, 0.133, 0.133]
MOVEABLE_COLOUR = [0.0, 0.0, 1.0]
OOI_COLOUR = [1.0, 0.843, 0.0]

FALL_POS_THRESH = 0.436332 # float('inf')
FALL_VEL_THRESH = 2.0 # float('inf')
CONTACT_THRESH = 1e-9

YCB_OBJECTS = {
	2: '002_master_chef_can',
	3: '003_cracker_box',
	4: '004_sugar_box',
	5: '005_tomato_soup_can',
	6: '006_mustard_bottle',
	7: '007_tuna_fish_can',
	8: '008_pudding_box',
	9: '009_gelatin_box',
	10: '010_potted_meat_can',
	11: '011_banana',
	19: '019_pitcher_base',
	21: '021_bleach_cleanser',
	24: '024_bowl',
	25: '025_mug',
	35: '035_power_drill',
	36: '036_wood_block'
}

#######
# PR2 #
#######

REST_LEFT_ARM = [1.5707963267948966, 1.5707963267948966, 0.0, -0.7853981633974483, 0.0, 0.0, 0.0]
WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]

BASE_LINK = -1

ARM_NAMES = ('left', 'right')

def arm_from_arm(arm): # TODO: rename
	assert (arm in ARM_NAMES)
	return '{}_arm'.format(arm)

def gripper_from_arm(arm):
	assert (arm in ARM_NAMES)
	return '{}_gripper'.format(arm)


PR2_GROUPS = {
	'base': ['x', 'y', 'theta'],
	'torso': ['torso_lift_joint'],
	'head': ['head_pan_joint', 'head_tilt_joint'],
	arm_from_arm('left'): ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
				 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
	arm_from_arm('right'): ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
				  'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
	gripper_from_arm('left'): ['l_gripper_l_finger_joint', 'l_gripper_r_finger_joint',
					 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_tip_joint'],
	gripper_from_arm('right'): ['r_gripper_l_finger_joint', 'r_gripper_r_finger_joint',
					  'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_tip_joint'],
	# r_gripper_joint & l_gripper_joint are not mimicked
}

HEAD_LINK_NAME = 'high_def_optical_frame' # high_def_optical_frame | high_def_frame | wide_stereo_l_stereo_camera_frame
# kinect - 'head_mount_kinect_rgb_optical_frame' | 'head_mount_kinect_rgb_link'

PR2_TOOL_FRAMES = {
	'left': 'l_gripper_tool_frame',  # l_gripper_palm_link | l_gripper_tool_frame
	'right': 'r_gripper_tool_frame',  # r_gripper_palm_link | r_gripper_tool_frame
	'head': HEAD_LINK_NAME,
}

PR2_GRIPPER_ROOTS = {
	'left': 'l_gripper_palm_link',
	'right': 'r_gripper_palm_link',
}

PR2_BASE_LINK = 'base_footprint'

##########
# Joints #
##########

def get_num_joints(body, sim):
	if sim is None:
		return p.getNumJoints(body)
	else:
		return sim.getNumJoints(body)

def get_joints(body, sim=None):
	return list(range(get_num_joints(body, sim)))

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
									 'qIndex', 'uIndex', 'flags',
									 'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
									 'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
									 'parentFramePos', 'parentFrameOrn', 'parentIndex'])

def get_joint_info(body, joint, sim):
	if sim is None:
		return JointInfo(*p.getJointInfo(body, joint))
	else:
		return JointInfo(*sim.getJointInfo(body, joint))

def get_joint_name(body, joint, sim):
	return get_joint_info(body, joint, sim).jointName.decode('UTF-8')

def joint_from_name(body, name, sim=None):
	for joint in get_joints(body, sim=sim):
		if get_joint_name(body, joint, sim) == name:
			return joint
	raise ValueError(body, name)

def joints_from_names(body, names, sim=None):
	return tuple(joint_from_name(body, name, sim) for name in names)

JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
									   'jointReactionForces', 'appliedJointMotorTorque'])

def get_joint_state(body, joint, sim):
	if sim is None:
		return JointState(*p.getJointState(body, joint))
	else:
		return JointState(*sim.getJointState(body, joint))

def get_joint_position(body, joint, sim=None):
	return get_joint_state(body, joint, sim).jointPosition

def get_joint_velocity(body, joint, sim=None):
	return get_joint_state(body, joint, sim).jointVelocity

def get_joint_reaction_force(body, joint, sim=None):
	return get_joint_state(body, joint, sim).jointReactionForces

def get_joint_torque(body, joint, sim=None):
	return get_joint_state(body, joint, sim).appliedJointMotorTorque

def get_joint_positions(body, joints, sim=None): # joints=None):
	return tuple(get_joint_position(body, joint, sim) for joint in joints)

def get_joint_velocities(body, joints, sim=None):
	return tuple(get_joint_velocity(body, joint, sim) for joint in joints)

#########
# Links #
#########

get_links = get_joints # Does not include BASE_LINK

def get_all_links(body, sim=None):
	return [BASE_LINK] + list(get_links(body, sim=sim))

def get_link_parent(body, link, sim):
	if link == BASE_LINK:
		return None
	return get_joint_info(body, link, sim).parentIndex

def get_all_link_parents(body, sim):
	return {link: get_link_parent(body, link, sim) for link in get_links(body, sim=sim)}

def get_all_link_children(body, sim):
	children = {}
	for child, parent in get_all_link_parents(body, sim).items():
		if parent not in children:
			children[parent] = []
		children[parent].append(child)
	return children

def get_link_children(body, link, sim):
	children = get_all_link_children(body, sim)
	return children.get(link, [])

def get_link_descendants(body, link, sim, test=lambda l: True):
	descendants = []
	for child in get_link_children(body, link, sim):
		if test(child):
			descendants.append(child)
			descendants.extend(get_link_descendants(body, child, sim, test=test))
	return descendants

def get_link_subtree(body, link, sim=None, **kwargs):
	return [link] + get_link_descendants(body, link, sim, **kwargs)

LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
									 'localInertialFramePosition', 'localInertialFrameOrientation',
									 'worldLinkFramePosition', 'worldLinkFrameOrientation'])

def get_link_state(body, link, sim=None):
	if sim is None:
		return LinkState(*p.getLinkState(body, link))
	else:
		return LinkState(*sim.getLinkState(body, link))

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])

def get_body_info(body, sim):
	if sim is None:
		return BodyInfo(*p.getBodyInfo(body, physicsClientId=CLIENT))
	else:
		return BodyInfo(*sim.getBodyInfo(body, physicsClientId=CLIENT))

def get_base_name(body, sim=None):
	return get_body_info(body, sim).base_name.decode(encoding='UTF-8')

def get_link_name(body, link, sim=None):
	if link == BASE_LINK:
		return get_base_name(body, sim)
	return get_joint_info(body, link, sim).linkName.decode('UTF-8')

def link_from_name(body, name, sim=None):
	if name == get_base_name(body, sim):
		return BASE_LINK
	for link in get_joints(body, sim):
		if get_link_name(body, link, sim) == name:
			return link
	raise ValueError(body, name)

########
# AABB #
########

AABB = namedtuple('AABB', ['lower', 'upper'])

def aabb_from_points(points):
	return AABB(np.min(points, axis=0), np.max(points, axis=0))

def aabb_union(aabbs):
	return aabb_from_points(np.vstack([aabb for aabb in aabbs]))

def get_subtree_aabb(body, root_link=BASE_LINK, sim=None):
	return aabb_union(get_aabb(body, link, sim) for link in get_link_subtree(body, root_link, sim))

def get_aabbs(body, sim=None):
	return [get_aabb(body, link=link, sim=sim) for link in get_all_links(body)]

def get_aabb(body, link=None, sim=None):
	# Note that the query is conservative and may return additional objects that don't have actual AABB overlap.
	# This happens because the acceleration structures have some heuristic that enlarges the AABBs a bit
	# (extra margin and extruded along the velocity vector).
	# Contact points with distance exceeding this threshold are not processed by the LCP solver.
	# AABBs are extended by this number. Defaults to 0.02 in Bullet 2.x
	#p.setPhysicsEngineParameter(contactBreakingThreshold=0.0, physicsClientId=CLIENT)
	if link is None:
		aabb = aabb_union(get_aabbs(body, sim=sim))
	else:
		if sim is None:
			aabb = p.getAABB(body, linkIndex=link)
		else:
			aabb = sim.getAABB(body, linkIndex=link)
	return aabb

##########
# Angles #
##########

FLOATS = [float, np.float128, np.float64, np.float32]

def normalize_angle(a):
	if (np.any(np.fabs(a) > 2*np.pi)):
		if (type(a) in FLOATS):
			a = np.fmod(a, 2*np.pi)
		else:
			r = np.where(np.fabs(a) > 2*np.pi)
			a[r[0]] = np.fmod(a[r[0]], 2*np.pi)
	while (np.any(a < -np.pi)):
		if (type(a) in FLOATS):
			a += 2*np.pi
		else:
			r = np.where(a < -np.pi)
			a[r[0]] += 2*np.pi
	while (np.any(a > np.pi)):
		if (type(a) in FLOATS):
			a -= 2*np.pi
		else:
			r = np.where(a > np.pi)
			a[r[0]] -= 2*np.pi
	return a

def shortest_angle_diff(af, ai):
	return normalize_angle(af - ai)

def shortest_angle_dist(af, ai):
	return np.fabs(shortest_angle_diff(af, ai))

########
# Misc #
########

def drawAABB(aabb, sim=None):
	if sim is None:
		sim = p

	aabbMin = aabb[0]
	aabbMax = aabb[1]

	f = [aabbMin[0], aabbMin[1], aabbMin[2]]
	t = [aabbMax[0], aabbMin[1], aabbMin[2]]
	sim.addUserDebugLine(f, t, [1, 0, 0])
	f = [aabbMin[0], aabbMin[1], aabbMin[2]]
	t = [aabbMin[0], aabbMax[1], aabbMin[2]]
	sim.addUserDebugLine(f, t, [0, 1, 0])
	f = [aabbMin[0], aabbMin[1], aabbMin[2]]
	t = [aabbMin[0], aabbMin[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [0, 0, 1])

	f = [aabbMin[0], aabbMin[1], aabbMax[2]]
	t = [aabbMin[0], aabbMax[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMin[0], aabbMin[1], aabbMax[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMin[1], aabbMin[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMin[1], aabbMin[2]]
	t = [aabbMax[0], aabbMax[1], aabbMin[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMax[1], aabbMin[2]]
	t = [aabbMin[0], aabbMax[1], aabbMin[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMin[0], aabbMax[1], aabbMin[2]]
	t = [aabbMin[0], aabbMax[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])

	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMin[0], aabbMax[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMax[0], aabbMin[1], aabbMax[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])
	f = [aabbMax[0], aabbMax[1], aabbMax[2]]
	t = [aabbMax[0], aabbMax[1], aabbMin[2]]
	sim.addUserDebugLine(f, t, [1, 1, 1])
