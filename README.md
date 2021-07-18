## Repository Architecture
- The `Planner` class runs the outer loop for WHCA*.
- `Planner` initialises a `CollisionChecker` object shared between itself and
all agents - it populates the immovable obstacles right from the get go.
- The idea is that `CollisionChecker` will also be updated with the current
positions and trajectories of movable agents for dynamic obstacle collision
checking.
- Each `Agent` will run it's own A\* search in `(x, y, t)` to compute a path.
	- For the object-of-interest, we'll implement a windowed
	(limited expansion) search.
	- For the other movable objects, we should do the same, or we could
	experiment with running a BFS to the same depth, and returning the
	"most open" solution?
	- Initially, the abstract space distance will just be the naive heuristic
	(Euclidean or Manhattan distance). Eventually we'll implement RRA\*.

### Helper Functions
- `geometry.hpp` contains a bunch of functions to aid the collision checker.
	- Functions to compute circle-rectangle and rectangle-rectangle
	intersections.

### Domain Notes
- All agents except the OOI cannot exit table bounds.
- For the time being, agents cannot rotate in place.
	- Next step: allow in-place rotation
	- After that: loop in the simulator and change actions to force vectors.
- If you look at a scene image from the `simplan/data/clutter_scenes` folder,
the positive X-axis goes right, positive Y-axis goes up.
- Planner (WHCA\*) terminates when the OOI has a collision-free path from its
initial start to goal, and all other agents have collision-free paths (where to
does not matter).
- For any agent, even if `start == goal`, we still run the search till depth
`w`, i.e. the `is_goal` check should return true if a state at depth `w+1` is
being expanded.
# pushplan
