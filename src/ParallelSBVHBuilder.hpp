#ifndef ADYPT_PARALLELSBVHBUILDER_HPP
#define ADYPT_PARALLELSBVHBUILDER_HPP

#include "AtomicAllocator.hpp"
#include "BinaryBVH.hpp"
#include <atomic>
#include <concurrentqueue.h>

class ParallelSBVHBuilder {
private:
	static constexpr uint32_t kSpatialBinNum = 32, kAllocatorChunkSize = 64;
	const Scene &m_scene;
	const BVHConfig &m_config;
	float m_min_overlap_area;

	struct Node {
		AABB aabb;
		uint32_t m_tri_index{};
		Node *left{}, *right{};
	};
	AtomicAllocator<Node, kAllocatorChunkSize> m_node_allocator;

	struct Reference {
		AABB m_aabb;
		uint32_t m_tri_index{};
	};
	AtomicAllocator<Reference, kAllocatorChunkSize> m_reference_allocator;

	struct Task {
	private:
		Node *m_node{};
		std::vector<Reference *> m_references;

	public:
		inline Task() = default;
		inline Task(Node *node, const std::vector<Reference *> &references) : m_node{node}, m_references{references} {}
		inline Task(Node *node, std::vector<Reference *> &&references)
		    : m_node{node}, m_references{std::move(references)} {}
		inline bool Empty() const { return !m_node; }
		std::array<Task, 2> Run(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator,
		                        LocalAllocator<Reference, kAllocatorChunkSize> *p_reference_allocator);
	};
	moodycamel::ConcurrentQueue<Task> m_task_queue;
	std::atomic_uint32_t m_task_count{};

	void push_initial_task();

public:
	ParallelSBVHBuilder(const BVHConfig &config, const Scene &scene)
	    : m_scene(scene), m_config(config), m_min_overlap_area{scene.GetAABB().GetArea() * 1e-5f} {}
	void Run();
};

#endif
