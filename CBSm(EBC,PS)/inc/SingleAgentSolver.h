#pragma once

#include "Instance.h"
#include "ConstraintTable.h"

// static int counter_pot = 0;
// static int counter_other = 0;

// static int counter_conf1 = 0;
// static int counter_conf2 = 0;
// static int counter_conf3 = 0;
// static int counter_conf4 = 0;
// static int counter_conf5 = 0;

class LLNode // low-level node
{
public:
	int location;
	int g_val;
	int h_val = 0;
	LLNode* parent;
	int timestep = 0;
	int num_of_conflicts = 0;
	int temp_previous_makespan = -1; // makespan of the parent node (can be adjusted when changing start/goal to landmarks)
	bool in_openlist = false;
	bool wait_at_goal; // the action is to wait at the goal vertex or not. This is used for >length constraints
	// the following is used to compare nodes in the OPEN list
	struct compare_node
	{
		// returns true if n1 > n2 (note -- this gives us *min*-heap).
		bool operator()(const LLNode* n1, const LLNode* n2) const
		{
			if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
			{
				if (n1->h_val == n2->h_val)
					return rand() % 2;
				return n1->h_val >= n2->h_val;
			}
			return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
		}
	};  // used by OPEN (heap) to compare nodes (top of the heap has min f-val, and then highest g-val)

	// the following is used to compare nodes in the FOCAL list
	// struct secondary_compare_node
	// {
	// 	bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
	// 	{
	// 		if (n1->num_of_conflicts == n2->num_of_conflicts)
	// 		{
	// 			if (n1->g_val == n2->g_val)
	// 			{
	// 				return rand() % 2 == 0;
	// 			}
	// 			return n1->g_val <= n2->g_val;  // break ties towards larger g_vals
	// 		}
	// 		return n1->num_of_conflicts >= n2->num_of_conflicts;  // n1 > n2 if it has more conflicts
	// 	}
	// };  // used by FOCAL (heap) to compare nodes (top of the heap has min number-of-conflicts)


	// the following is used to compare nodes in the FOCAL list with potential search
	struct secondary_compare_node
	{
		bool operator()(const LLNode* n1, const LLNode* n2) const // returns true if n1 > n2
		{
			// if (potential_bound == 0)
			// {
			// 	return n1->num_of_conflicts >= n2->num_of_conflicts;
			// }
			int n1_priority = -1;
			if (n1->h_val==0 and n1->g_val<n1->temp_previous_makespan)
			{
				n1_priority = 0;
			}
			else if (n1->h_val==0 and n1->g_val>=n1->temp_previous_makespan)
			{
				n1_priority = 1;
			}
			else if (n1->h_val>0 and n1->g_val<n1->temp_previous_makespan)
			{
				n1_priority = 2;
			}
			else if (n1->h_val>0 and n1->g_val>=n1->temp_previous_makespan)
			{
				n1_priority = 3;
			}
			else
			{
				n1_priority = 4;
			}
			int n2_priority = -1;
			if (n2->h_val==0 and n2->g_val<n2->temp_previous_makespan)
			{
				n2_priority = 0;
			}
			else if (n2->h_val==0 and n2->g_val>=n2->temp_previous_makespan)
			{
				n2_priority = 1;
			}
			else if (n2->h_val>0 and n2->g_val<n2->temp_previous_makespan)
			{
				n2_priority = 2;
			}
			else if (n2->h_val>0 and n2->g_val>=n2->temp_previous_makespan)
			{
				n2_priority = 3;
			}
			else
			{
				n2_priority = 4;
			}

			// cout << counter_conf << " " << counter_pot << " " << counter_other << endl;
			// cout << counter_conf1 << " " << counter_conf2 << " " << counter_conf3 << " " << counter_conf4 << " " << counter_conf5 << " " << counter_pot << " " << counter_other << endl;


			if(n1_priority==n2_priority)
			{
				//cout << "same priority: " << n1_priority << endl;
				switch (n1_priority)
				{
				case 0:
					if(n1->g_val == n2->g_val)
					{
						// counter_conf++;
						// counter_conf1++;
						return n1->num_of_conflicts >= n2->num_of_conflicts;
					}
					// counter_other++;
					return n1->g_val >= n2->g_val;
				case 1:
					// counter_conf++;
					// counter_conf2++;
					return n1->num_of_conflicts >= n2->num_of_conflicts;
				case 2:
					if(static_cast<float>(n1->h_val)/static_cast<float>(n1->temp_previous_makespan-n1->g_val) == static_cast<float>(n2->h_val)/static_cast<float>(n2->temp_previous_makespan-n2->g_val))
					{
						// counter_conf++;
						// counter_conf3++;
						// cout << n1->h_val << " " << n1->g_val << " " << n1->temp_previous_makespan << endl;
						// cout << n2->h_val << " " << n2->g_val << " " << n2->temp_previous_makespan << endl;
						// cout << endl;
						// if (n1->h_val == n2->h_val)
						return n1->num_of_conflicts >= n2->num_of_conflicts;
						// return n1->h_val >= n2->h_val;
					}
					// counter_pot++;
					return static_cast<float>(n1->h_val)/static_cast<float>(n1->temp_previous_makespan-n1->g_val) >= static_cast<float>(n2->h_val)/static_cast<float>(n2->temp_previous_makespan-n2->g_val);
				case 3:
					if(n1->h_val == n2->h_val)
					{
						// counter_conf++;
						// counter_conf4++;
						return n1->num_of_conflicts >= n2->num_of_conflicts;
					}
					// counter_other++;
					return n1->h_val >= n2->h_val;
				case 4:
					// counter_conf++;
					// counter_conf5++;
					return n1->num_of_conflicts >= n2->num_of_conflicts;
				}
			}
			else
			{
				// counter_other++;
				//cout << "different priority: n1: " << n1_priority << " n2: " << n2_priority << endl;
				return n1_priority > n2_priority;
			}
			return false;
		}
	};


	LLNode() : location(0), g_val(0), h_val(0), parent(nullptr), timestep(0), num_of_conflicts(0), temp_previous_makespan(-1), in_openlist(false), wait_at_goal(false) {}

	LLNode(int location, int g_val, int h_val, LLNode* parent, int timestep, int num_of_conflicts = 0, int temp_previous_makespan=-1, bool in_openlist = false) :
		location(location), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
		num_of_conflicts(num_of_conflicts), temp_previous_makespan(temp_previous_makespan), in_openlist(in_openlist), wait_at_goal(false) {}

	inline double getFVal() const { return g_val + h_val; }
	void copy(const LLNode& other)
	{
		location = other.location;
		g_val = other.g_val;
		h_val = other.h_val;
		parent = other.parent;
		timestep = other.timestep;
		num_of_conflicts = other.num_of_conflicts;
		wait_at_goal = other.wait_at_goal;
	}
};


class SingleAgentSolver
{
public:
	uint64_t num_expanded = 0;
	uint64_t num_generated = 0;

	double runtime_build_CT = 0; // runtime of building constraint table
	double runtime_build_CAT = 0; // runtime of building conflict avoidance table
	
	int temp_previous_makespan; // makespan of the parent node (can be adjusted when changing start/goal to landmarks)
	int start_location;
	int goal_location;
	vector<int> my_heuristic;  // this is the precomputed heuristic for this agent
	int compute_heuristic(int from, int to) const  // compute admissible heuristic between two locations
	{
		return max(get_DH_heuristic(from, to), instance.getManhattanDistance(from, to));
	}
	const Instance& instance;

	virtual Path findPath(const CBSNode& node, const ConstraintTable& initial_constraints,
		const vector<Path*>& paths, int agent, int lower_bound) = 0;
	virtual int getTravelTime(int end, const ConstraintTable& constraint_table, int upper_bound) = 0;
	virtual string getName() const = 0;

	list<int> getNextLocations(int curr) const; // including itself and its neighbors
	list<int> getNeighbors(int curr) const { return instance.getNeighbors(curr); }

	// int getStartLocation() const {return instance.start_locations[agent]; }
	// int getGoalLocation() const {return instance.goal_locations[agent]; }

	SingleAgentSolver(const Instance& instance, int agent) :
		instance(instance), //agent(agent), 
		start_location(instance.start_locations[agent]),
		goal_location(instance.goal_locations[agent])
	{
		compute_heuristics();
	}

	virtual ~SingleAgentSolver() {}

protected:
	void compute_heuristics();
	int get_DH_heuristic(int from, int to) const { return abs(my_heuristic[from] - my_heuristic[to]); }
};

