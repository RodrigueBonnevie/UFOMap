
#ifndef UFOMAP_DYNAMIC_H
#define UFOMAP_DYNAMIC_H

#include <ufomap/octree_dynamic.h>

#include <cstdint>

namespace ufomap
{
/**
 * @brief states of observed nodes in session
 *
 */
struct session_node_state {
    bool previous_state_free = false;
    bool previous_state_occupied = false;
    bool current_state_free = false;
    bool current_state_occupied = false;
    OccupancyNodeDynamic& node; 
};

/**
 * @brief dynamic parameter bundle in node
 *
 */
struct Dynamic
{
	int alpha_entry;
	int beta_entry;
	int alpha_exit;
	int beta_exit;

	Dynamic() : alpha_entry(1), beta_entry(2), 
                alpha_exit(1), beta_exit(2)
	{
	}

	Dynamic(int a_en, int b_en, int a_ex, int b_ex) : alpha_entry(a_en), beta_entry(b_en), 
                alpha_exit(a_ex), beta_exit(b_ex)
	{
	}


	Dynamic(const Dynamic& other) : alpha_entry(other.alpha_entry), beta_entry(other.beta_entry), 
                                    alpha_exit(other.alpha_exit), beta_exit(other.beta_exit)

	{
	}

	Dynamic& operator=(const Dynamic& rhs)
	{
		alpha_entry = rhs.alpha_entry;
		beta_entry = rhs.beta_entry;
		alpha_exit = rhs.alpha_exit;
		beta_exit = rhs.beta_exit;
		return *this;
	}

	inline bool operator==(const Dynamic& other) const
	{
		return alpha_entry == other.alpha_entry &&
                beta_entry == other.beta_entry &&
                alpha_exit == other.alpha_exit &&
                beta_exit == other.beta_exit;
	}

	inline bool operator!=(const Dynamic& other) const
	{
		return alpha_entry != other.alpha_entry ||
                beta_entry != other.beta_entry ||
                alpha_exit != other.alpha_exit ||
                beta_exit != other.beta_exit;
	}
};
}  // namespace ufomap

#endif  // UFOMAP_COLOR_H
