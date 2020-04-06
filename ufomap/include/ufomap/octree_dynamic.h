#ifndef UFOMAP_OCTREE_DYNAMIC_H
#define UFOMAP_OCTREE_DYNAMIC_H

#include <ufomap/node.h>
#include <ufomap/octree_base.h>
//#include <ufomap/octree.h>
//#include <ufomap/types.h>

#include <random>
#include <map>
namespace ufomap
{
/**
 * @brief states of observed nodes in session
 *
 */
struct session_node_state {
    bool previous_state_free; 
    bool previous_state_occupied;
    bool current_state_free;
    bool current_state_occupied;
    const Node<OccupancyNodeDynamic>* node; 

	session_node_state(const Node<OccupancyNodeDynamic>* node_in) :
		previous_state_free(false),
		previous_state_occupied(false),
		current_state_free(false),
		current_state_occupied(false),
		node(node_in)
	{
	}
};

class OctreeDynamic : public OctreeBase<OccupancyNodeDynamic>
{
public:
	//
	// Constructors and destructors
	//

	/**
	 * @brief Default constructor
	 *
	 * @param resolution
	 * @param depth_levels
	 * @param automatic_pruning
	 * @param occupancy_thres
	 * @param free_thres
	 * @param prob_hit
	 * @param prob_miss
	 * @param clamping_thres_min
	 * @param clamping_thres_max
	 */
	OctreeDynamic(float resolution = 0.1, unsigned int depth_levels = 16,
						bool automatic_pruning = false, bool prune_consider_dynaimc = true,
						float occupancy_thres = 0.5, float free_thres = 0.5, float prob_hit = 0.7,
						float prob_miss = 0.4, float clamping_thres_min = 0.1192,
						float clamping_thres_max = 0.971);

	OctreeDynamic(const std::string& filename);

	/**
	 * @brief Copy constructor
	 *
	 * @param other
	 */
	OctreeDynamic(const OctreeDynamic& other);

	/**
	 * @brief Destructor
	 *
	 */
	virtual ~OctreeDynamic()
	{
	}

	//
	// Tree type
	//

	virtual std::string getTreeType() const
	{
		return "OctreeDynamic";
	}

	virtual std::string getTreeTypeOctomap() const override
	{
		return "DynamicOcTree";
	}


protected:
    //
    // Dynamic
    //
    std::map<const Node<OccupancyNodeDynamic>*, session_node_state> changed_nodes_in_session_;
	int session_number_ = 0;

	//
	// Node collapsible
	//

	//
	// Update node
	//

	void update_dynamic_parameters(int session_number);
	Node<OccupancyNodeDynamic> updateNodeValue(const Code& code, float logit_update);
//{
//    Node<OccupancyNodeDynamic> node = getNode(code);
//    Node<OccupancyNodeDynamic> retrun_node;
//        retrun_node = node;
//			const Node<OccupancyNodeDynamic>* groda = &node;
//    session_node_state changed_node_struct(&node);
//    bool previously_unknown = isUnknown(node); // If the node is previously unknown nothing shuld happen to the dynamic estimation of the node
//    if (!changed_nodes_in_session_.count(groda) && !previously_unknown)  // New observation  TODO: Double check that this key is unique
//    {
//        if (isFree(node))
//        {
//            changed_node_struct.previous_state_free = true;
//        }
//        if (isOccupied(node))
//        {  // Will have to be changed if voxels fade to unkown in the future
//            changed_node_struct.previous_state_occupied = true;
//        }
//        changed_nodes_in_session_.insert(std::pair<Node<OccupancyNodeDynamic>*, session_node_state>
//                                                            (&node, changed_node_struct));
//    }
//
//    // static update
//    Node<OccupancyNodeDynamic> retrun_node;
//    if ((0 <= logit_update && node.node->logit >= clamping_thres_max_log_) ||
//            (0 >= logit_update && node.node->logit <= clamping_thres_min_log_))
//    {
//        //return node;
//        retrun_node = node;
//    }
//    retrun_node =  updateNodeValueRecurs(code, logit_update, root_, depth_levels_).first;
//
//    // Dynamic continued
//    if (!previously_unknown){
//        if (isFree(node)){
//            changed_nodes_in_session_[groda];//.current_state_free = true;
//            //changed_nodes_in_session_[&node].current_state_occupied = false;
//        }
//        if (isOccupied(node)){  // Will have to be changed if voxels fade to unkown in the future
//            //changed_nodes_in_session_[&node].current_state_occupied = true;
//            //changed_nodes_in_session_[&node].current_state_free = false;
//        }
//    }   
//    return retrun_node;
//};

float p_exit(Node<OccupancyNodeDynamic>* node)
{
	return float(node->node->dynamic.alpha_entry)/node->node->dynamic.beta_entry;
}

float p_entry(Node<OccupancyNodeDynamic>* node)
{
	return float(node->node->dynamic.alpha_exit)/node->node->dynamic.beta_exit;
}
	//
	// Read/write
	//

	virtual bool binarySupport() const override
	{
		return false;
	}

protected:
	bool prune_consider_dynamic_ = false;

	std::random_device dev_;
	std::mt19937 rng_;
	std::uniform_int_distribution<std::mt19937::result_type> dist_;
};

} //namespace ufomap

#endif  // UFOMAP_OCTREE_DYNAMIC_H