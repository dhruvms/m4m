## Repository Architecture
- The `Planner` class runs the outer loop for WHCA*
- It also uses the SMPL Robot Model to plan a sequence of (first-order only)
rearrangement sequences
- The `Robot` class can sample object-centric pushes
	- A bunch of parameters are randomised: robot link, push distance, EE pose
	parameters
- `Robot` also runs ARA* between the home state and push start state

### Intended Algorithm
1. Run MAPF planner
2. Find "first-order" interactions
3. Sample (straight line) pushes for each object
	- Parameters: robot link, move distance, EE pose during push
4. Find valid push via simulations
5. Create and verify rearrangment plan
	1. Sequence pushes
		- Plan to push location, append push
	2. Execute MAPF robot trajectory to grasp
6. If SUCCESS, done
7. Else, repeat from Step 1 with new object positions

### Domain Notes
- All agents except the OOI cannot exit table bounds.
- For the time being, agents cannot rotate in place.
	- Allow in-place rotation?
- If you look at a scene image from the `simplan/data/clutter_scenes` folder,
the positive X-axis goes right, positive Y-axis goes up.
- Planner (WHCA\*) runs in two phases:
	- Phase 1 terminates when robot "collides" with OOI
	- Phase 2 terminates when the OOI has been extracted
- For any agent, even if `start == goal`, we still run the search till depth
`w`, i.e. the `is_goal` check should return true if a state at depth `w+1` is
being expanded.
