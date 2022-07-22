#ifndef ADYPT_PARALLELSBVHBUILDER_HPP
#define ADYPT_PARALLELSBVHBUILDER_HPP

#include "AtomicAllocator.hpp"
#include "BinaryBVH.hpp"
#include <atomic>
#include <cfloat>
#include <concurrentqueue.h>
#include <utility>

class ParallelSBVHBuilder {
private:
	static constexpr uint32_t kSpatialBinNum = 32, kAllocatorChunkSize = 64;
	const Scene &m_scene;
	const BVHConfig &m_config;
	float m_min_overlap_area;
	uint32_t m_local_run_threshold;

	struct Node {
		AABB aabb;
		Node *left{}, *right{};
		uint32_t tri_idx{};
	} m_root{};
	AtomicAllocator<Node, kAllocatorChunkSize> m_node_allocator;

	struct Reference {
		AABB aabb;
		uint32_t tri_idx{};
	};

	template <uint32_t DIM> inline static bool reference_cmp(const Reference &l, const Reference &r);
	template <uint32_t DIM> inline static void sort_references(std::vector<Reference> *references);
	inline std::tuple<Reference, Reference> split_reference(const Reference &ref, uint32_t dim, float pos) const;

	std::atomic_uint32_t m_node_count{};
	struct Task {
	private:
		const ParallelSBVHBuilder *m_p_builder{};
		Node *m_node{};
		std::vector<Reference> m_references;
		uint32_t m_depth;

		struct ObjectSplit {
			AABB left_aabb, right_aabb;
			uint32_t dim{};
			float pos{}, sah{FLT_MAX};
		};
		struct SpatialSplit {
			uint32_t dim{};
			float pos{}, sah{FLT_MAX};
		};

		template <uint32_t DIM> inline void _find_spatial_split_dim(SpatialSplit *p_ss);
		inline SpatialSplit find_spatial_split();
		inline std::tuple<Task, Task> perform_spatial_split(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator,
		                                                    const SpatialSplit &ss);

		template <uint32_t DIM> inline void _find_object_split_dim(ObjectSplit *p_os);
		inline ObjectSplit find_object_split();
		inline std::tuple<Task, Task> perform_object_split(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator,
		                                                   const ObjectSplit &os);

		inline void perform_leaf() { m_node->tri_idx = m_references.front().tri_idx; }

	public:
		inline Task() = default;
		inline Task(const ParallelSBVHBuilder *p_builder, Node *node, const std::vector<Reference> &references,
		            uint32_t depth)
		    : m_p_builder{p_builder}, m_node{node}, m_references{references}, m_depth{depth} {}
		inline Task(const ParallelSBVHBuilder *p_builder, Node *node, std::vector<Reference> &&references,
		            uint32_t depth)
		    : m_p_builder{p_builder}, m_node{node}, m_references{std::move(references)}, m_depth{depth} {}
		Task(const Task &r) = delete;
		Task &operator=(const Task &r) = delete;

		Task(Task &&r) = default;
		Task &operator=(Task &&r) = default;

		inline static bool PairEmpty(const std::tuple<Task, Task> &p) { return std::get<0>(p).Empty(); }
		inline bool Empty() const { return !m_node; }
		std::tuple<Task, Task> Run(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator);
		inline uint32_t GetReferenceCount() const { return m_references.size(); }
	};
	moodycamel::ConcurrentQueue<Task> m_task_queue;
	std::atomic_uint32_t m_task_count{};

	void push_root_task();
	void local_run_task(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator, Task &&task);
	uint32_t fetch_result(Node *cur, std::vector<BinaryBVH::Node> *p_nodes, uint32_t *p_leaf_count);

public:
	ParallelSBVHBuilder(const BVHConfig &config, const Scene &scene)
	    : m_scene(scene), m_config(config), m_min_overlap_area{scene.GetAABB().GetArea() * 1e-5f},
	      m_local_run_threshold{std::clamp((uint32_t)scene.GetTriangles().size() >> 12u, 16u, 256u)} {}
	void Run();
	void FetchResult(std::vector<BinaryBVH::Node> *p_nodes, uint32_t *p_leaf_count);
};

#endif
