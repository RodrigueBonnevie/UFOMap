#include <ufomap/octree_dynamic.h>

#include <sstream>

namespace ufomap
{
//
// Constructors and destructors
//

OctreeDynamic::OctreeDynamic(float resolution, unsigned int depth_levels, bool automatic_pruning,
                                bool prune_consider_dynamic, float occupancy_thres, float free_thres,
                                float prob_hit, float prob_miss, float clamping_thres_min,
                                float clamping_thres_max)
	: OctreeBase(resolution, depth_levels, automatic_pruning, occupancy_thres, free_thres,
							 prob_hit, prob_miss, clamping_thres_min, clamping_thres_max)
	, prune_consider_dynamic_(prune_consider_dynamic)
	, rng_(dev_())
	, dist_(0, 10)
{
}

OctreeDynamic::OctreeDynamic(const std::string& filename) : OctreeDynamic()
{
	read(filename);
}

OctreeDynamic::OctreeDynamic(const OctreeDynamic& other)
	: OctreeDynamic(other.resolution_, other.depth_levels_, other.automatic_pruning_enabled_,
							other.prune_consider_dynamic_, other.getOccupancyThres(),
							other.getFreeThres(), other.getProbHit(), other.getProbMiss(),
							other.getClampingThresMin(), other.getClampingThresMax())
{
	// TODO: Is correct?
	std::stringstream s(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
	other.write(s);
	read(s);
}

//
// Insertion

Node<OccupancyNodeDynamic> updateNodeValue(const Code& code, float logit_update)
{
    Node<OccupancyNodeDynamic> node = getNode(code);
    session_node_state changed_node_struct;
    bool previously_unknown = isUnknown(*node); // If the node is previously unknown nothing shuld happen to the dynamic estimation of the node
    if (!changed_nodes_in_session_.count(node) && !previously_unknown)  // New observation  TODO: Double check that this key is unique
    {
        if (isFree(*node))
        {
            changed_node_struct.previous_state_free = true;
        }
        if (isOccupied(*node))
        {  // Will have to be changed if voxels fade to unkown in the future
            changed_node_struct.previous_state_occupied = true;
        }
        changed_node_struct.node = *node.node;
        changed_nodes_in_session_.insert(std::pair<const OccupancyNodeDynamic&, session_node_state>
                                                            (*node.node, changed_node_struct));
    }

    // static update
    Node<OccupancyNodeDynamic> retrun_node;
    if ((0 <= logit_update && node.node->logit >= clamping_thres_max_log_) ||
            (0 >= logit_update && node.node->logit <= clamping_thres_min_log_))
    {
        //return node;
        retrun_node = node;
    }
    retrun_node =  updateNodeValueRecurs(code, logit_update, root_, depth_levels_).first;

    // Dynamic continued
    if (!previously_unknown){
        if (isFree(*node)){
            changed_nodes_in_session_[node].current_state_free = true;
            changed_nodes_in_session_[node].current_state_occupied = false;
        }
        if (isOccupied(*node)){  // Will have to be changed if voxels fade to unkown in the future
            changed_nodes_in_session_[node].current_state_occupied = true;
            changed_nodes_in_session_[node].current_state_free = false;
        }
    }   
    return retrun_node;

}


//void OctreeDynamic::insertNoCheck(const Code& code, bool hit, unsigned int depth)
//{
//	const OccupancyNode* node = search(code, depth, true); 
//    OccupancyNode* node_unconst = const_cast<OccupancyNode*> (node);  // May be better to overload search function to return non const
//    // Dynamic update
//    node_unconst->session_last_seen = session_number_;
//    session_node changed_node_struct;
//    bool previously_unknown; // this isn't right the first session, or you can say that is assumes another initial value to the dynamic parameters
//    previously_unknown = isUnknown(*node); // If the node is previously unknown nothing shuld happen to the dynamic estimation of the node
//    if (!changed_nodes_session_.count(node_unconst) && !previously_unknown){  // New observation  TODO: Double check that this key is unique
//        if (isFree(*node)){
//            changed_node_struct.previous_state_free = true;
//        }
//        if (isOccupied(*node)){  // Will have to be changed if voxels fade to unkown in the future
//            changed_node_struct.previous_state_occupied = true;
//        }
//        changed_node_struct.node = node_unconst;
//        changed_nodes_session_.insert(std::pair<OccupancyNode*, session_node>(node_unconst, changed_node_struct));
//    }
//    // Non dynamic update
//    float update = hit ? prob_hit_log_ : prob_miss_log_;  
//    if (!((0 <= update && node->logit >= clamping_thres_max_log_) ||
//			(0 >= update && node->logit <= clamping_thres_min_log_)) )
//	{
//        insertNoCheckRecurs(root_, depth_levels_, code, update, depth);
//	}
//
//    // dynamic update continued
//    if (!previously_unknown){
//        if (isFree(*node)){
//            changed_nodes_session_[node_unconst].current_state_free = true;
//            changed_nodes_session_[node_unconst].current_state_occupied = false;
//        }
//        if (isOccupied(*node)){  // Will have to be changed if voxels fade to unkown in the future
//            changed_nodes_session_[node_unconst].current_state_occupied = true;
//            changed_nodes_session_[node_unconst].current_state_free = false;
//        }
//    }   
//
//}
//
//void OctreeDynamic::update_dynamic_parameters(int session_number){
//    // Should be run at end of session.
//    // Updating alpha and beta variables
//    // state_prev shoud be the state from the previous session
//    // state_now could be the latest measured state in the session
//    fprintf(stderr, "starting update_dynamic_parameters \n");
//    session_number_ = session_number;
//    int padda = 0;
//    for (auto it = changed_nodes_session_.begin(); it !=changed_nodes_session_.end(); ++it){
//        OccupancyNode* node = it->second.node;
//
//        if (it->second.previous_state_free){
//            ++ node->beta_entry; 
//            if (it->second.current_state_occupied){
//                ++ node->alpha_entry;
//            }
//        }
//        if (it->second.previous_state_occupied){
//            ++ node->beta_exit;
//            if (it->second.current_state_free){
//                ++ node->alpha_exit;
//            }
//        }
//        // Updating markov parameters
//        node->p_exit = float(node->alpha_exit)/node->beta_exit;
//        node->p_entry = float(node->alpha_entry)/node->beta_entry;
//
//        if ( node->p_exit > 1.0 || node->p_exit < 0.0){
//            fprintf(stderr,"Illegal value of p_exit : %f \n ", node->p_exit);
//            fprintf(stderr, "alpha = %d, beta = %d \n",node->alpha_exit, node->beta_exit);
//            fprintf(stderr, "p_occ = %d, p_free = %d, c_occ = %d, c_free = %d \n", it->second.previous_state_occupied,
//                    it->second.previous_state_free,
//                    it->second.current_state_occupied,
//                    it->second.current_state_free);
//            fprintf(stderr,"\n");
//        }
//        if (node->p_entry > 1.0 || node->p_entry < 0.0){
//            fprintf(stderr,"Illegal value of p_entry : %f \n", node->p_entry);
//            fprintf(stderr, "alpha = %d, beta = %d \n",node->alpha_entry, node->beta_entry);
//            fprintf(stderr, "p_occ = %d, p_free = %d, c_occ = %d, c_free = %d \n", it->second.previous_state_occupied,
//                    it->second.previous_state_free,
//                    it->second.current_state_occupied,
//                    it->second.current_state_free);
//            fprintf(stderr,"\n");
//        }
//        ++padda;
//    }
//    changed_nodes_session_.clear();
//    fprintf(stderr,"Number of updated dynamic parameters: %d \n", padda);
//}
}  // namespace ufomap
