#pragma once

#include "common.h"
#include "CBSNode.h"

//Amir: table for each agent for each high level node.
class ConstraintTable
{
public:
	int length_min = 0; //lower bound for the agent's path length
	int length_max = MAX_TIMESTEP; //upper bound for the agent's path length
	int goal_location;
	int latest_timestep = 0; // No negative constraints after this timestep.
	size_t num_col;  //the number of columns of the map.
	size_t map_size; //the map size.
	size_t cat_size; //the vector size. Number of timesteps recorded in the CAT.

	int getHoldingTime(); // the earliest timestep that the agent can hold its goal location (The agent can't get to its goal before this time)

	// void clear(){ct.clear(); cat_small.clear(); cat_large.clear(); landmarks.clear(); length_min = 0, length_max = INT_MAX; latest_timestep = 0;}

	//veryex constraint
	bool constrained(size_t loc, int t) const;
	//edge constraint (include where and when not to be + from where not to do the move)
	bool constrained(size_t curr_loc, size_t next_loc, int next_t) const;
	//return 1 if the agent can't step from curr_id to next_id at curr_timestep to next_timestep, else 0.
	int getNumOfConflictsForStep(size_t curr_id, size_t next_id, int next_timestep) const;
	
	size_t getNumOfPositiveConstraintSets() const {return positive_constraint_sets.size(); }
	
	bool updateUnsatisfiedPositiveConstraintSet(const list<int>& old_set, list<int>& new_set, int location, int timestep) const;
	ConstraintTable() = default;
	ConstraintTable(size_t num_col, size_t map_size, int goal_location = -1) : goal_location(goal_location), num_col(num_col), map_size(map_size) {}
	ConstraintTable(const ConstraintTable& other) { copy(other); }

	void copy(const ConstraintTable& other);
	void build(const CBSNode& node, int agent); // build the constraint table for the given agent at the given node
	void buildCAT(int agent, const vector<Path*>& paths, size_t cat_size); // build the conflict avoidance table

	void insert2CT(size_t loc, int t_min, int t_max); // insert a vertex constraint to the constraint table
	void insert2CT(size_t from, size_t to, int t_min, int t_max); // insert an edge constraint to the constraint table

	size_t getNumOfLandmarks() const { return landmarks.size(); }
	unordered_map<size_t, size_t> getLandmarks() const { return landmarks; }
	list<pair<int, int> > decodeBarrier(int B1, int B2, int t);
protected:
	// Constraint Table (CT)
	unordered_map<size_t, list<pair<int, int> > > ct; // location -> time range, or edge -> time range

	unordered_map<size_t, size_t> landmarks; // <timestep, location>: the agent must be at the given location at the given timestep

	//list of sets of positive constraints that agent need to satisfy at least one constraint of each set (like CNF).
	//The front of a set has the lowest timestep and the back has the highest timestep.
	vector< list<pair<int, int> > > positive_constraint_sets; // a vector of positive constraint sets, each of which is a sorted list of <location, timestep> pair.

	void insertLandmark(size_t loc, int t); // insert a landmark, i.e., the agent has to be at the given location at the given timestep

	inline size_t getEdgeIndex(size_t from, size_t to) const { return (1 + from) * map_size + to; }

private:
	size_t map_size_threshold = 10000;
	//AMIR: vector of lists of location/edge indices which are occupied by agents at a certain time.
	vector<list<size_t> > cat_large; // conflict avoidance table for large maps
	//AMIR: vector of map-states which represent the locations that occupied by agents with a true value.
	vector<vector<bool> > cat_small; // conflict avoidance table for small maps

	//AMIR: The vectors are ordered by time from start to end.
	//AMIR: Each CAT do not consider the agent that holds it (but only the rest of them).
};

