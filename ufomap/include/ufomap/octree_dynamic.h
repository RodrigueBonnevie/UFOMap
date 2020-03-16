#ifndef UFOMAP_OCTREE_DYNAMIC_H
#define UFOMAP_OCTREE_DYNAMIC_H

#include <ufomap/node.h>
#include <ufomap/octree_base.h>
#include <ufomap/octree.h>
#include <ufomap/types.h>

#include <random>
#include <map>
namespace ufomap
{
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

	//
	// Insertion
	//

	//void insertPointCloud(const Point3& sensor_origin, const PointCloud& cloud,
	//											float max_range = -1);

	//void insertPointCloudDiscrete(const Point3& sensor_origin, const PointCloud& cloud,
 //                                           float max_range = -1, bool super_speed = false,
 //                                           unsigned int depth = 0);

	//void insertPointCloud(const Point3& sensor_origin, const PointCloud& cloud,
 //                                           const Pose6& frame_origin, float max_range = -1)
	//{
	//	PointCloudRGB cloud_transformed(cloud);
	//	cloud_transformed.transform(frame_origin);
	//	insertPointCloud(sensor_origin, cloud_transformed, max_range);
	//}

	//void insertPointCloudDiscrete(const Point3& sensor_origin, const PointCloudRGB& cloud,
 //                                       const Pose6& frame_origin, float max_range = -1,
 //                                       bool super_speed = false, unsigned int depth = 0)
	//{
	//	PointCloudRGB cloud_transformed(cloud);
	//	cloud_transformed.transform(frame_origin);
	//	insertPointCloudDiscrete(sensor_origin, cloud_transformed, max_range, super_speed,
	//													 depth);
	//}

	//
	// Set node color
	//


protected:
    //
    // Dynamic
    //
    std::map<OccupancyNodeDynamic*, session_node_state> changed_nodes_in_session_;

	//
	// Node collapsible
	//

	virtual bool
	isNodeCollapsible(const std::array<OccupancyNodeDynamic, 8>& children) const override;

	virtual bool isNodeCollapsible(
			const std::array<InnerNode<OccupancyNodeDynamic>, 8>& children) const override;

	//
	// Update node
	//
	Node<OccupancyNodeDynamic> updateNodeValue(const Code& code, float logit_update) override;

	virtual bool updateNode(InnerNode<OccupancyNodeDynamic>& node,
													const std::array<OccupancyNodeDynamic, 8>& children,
													unsigned int depth) override;

	virtual bool updateNode(InnerNode<OccupancyNodeDynamic>& node,
													const std::array<InnerNode<OccupancyNodeDynamic>, 8>& children,
													unsigned int depth) override;

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
}  // namespace ufomap

#endif  // UFOMAP_OCTREE_DYNAMIC_H