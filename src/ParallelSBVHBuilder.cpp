#include "ParallelSBVHBuilder.hpp"

#include <future>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

void ParallelSBVHBuilder::Run() {
	push_initial_task();

	std::vector<std::future<void>> workers(std::thread::hardware_concurrency());
	for (auto &worker : workers) {
		worker = std::async([this]() -> void {
			moodycamel::ConsumerToken consumer_token{m_task_queue};
			moodycamel::ProducerToken producer_token{m_task_queue};
			auto node_allocator = LocalAllocator{m_node_allocator};
			auto reference_allocator = LocalAllocator{m_reference_allocator};

			Task task;
			while (m_task_count.load()) {
				if (m_task_queue.try_dequeue(consumer_token, task)) {
					int32_t delta = -1;
					auto new_tasks = task.Run(&node_allocator, &reference_allocator);
					for (auto &new_task : new_tasks) {
						if (new_task.Empty())
							continue;
						++delta;
						m_task_queue.enqueue(producer_token, new_task);
					}
					m_task_count.fetch_add(delta);
				}
			}
		});
	}
}

void ParallelSBVHBuilder::push_initial_task() {
	Node *node;
	std::vector<Reference *> references;
	{
		auto allocator = LocalAllocator{m_node_allocator};
		node = allocator.Alloc();
	}
	{
		auto allocator = LocalAllocator{m_reference_allocator};
		references.reserve(m_scene.GetTriangles().size());
		for (uint32_t i = 0; i < m_scene.GetTriangles().size(); ++i) {
			Reference *ref = allocator.Alloc();
			ref->m_tri_index = i;
			ref->m_aabb = m_scene.GetTriangles()[i].GetAABB();
			references.push_back(ref);
		}
	}
	m_task_count.store(1);
	m_task_queue.enqueue(Task{node, std::move(references)});
}

std::array<ParallelSBVHBuilder::Task, 2>
ParallelSBVHBuilder::Task::Run(LocalAllocator<Node, kAllocatorChunkSize> *p_node_allocator,
                               LocalAllocator<Reference, kAllocatorChunkSize> *p_reference_allocator) {
	spdlog::info("Run task with {} references", m_references.size());
	return {Task{}, Task{}};
}
