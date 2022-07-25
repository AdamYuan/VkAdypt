#ifndef ADYPT_PARALLELSBVHBUILDER_HPP
#define ADYPT_PARALLELSBVHBUILDER_HPP

#include "AtomicAllocator.hpp"
#include "BinaryBVH.hpp"
#include <atomic>
#include <cfloat>
#include <concurrentqueue.h>
#include <condition_variable>
#include <functional>
#include <future>
#include <optional>
#include <utility>

class ParallelSBVHBuilder {
private:
	const uint32_t kThreadCount;
	static constexpr uint32_t kSpatialBinNum = 32, kObjectBinNum = 32;
	static constexpr uint32_t kParallelForBlockSize = 64;
	static constexpr uint32_t kLocalRunThreshold = 512;
	static constexpr uint32_t kAtomicAllocatorChunk = 256;

	const Scene &m_scene;
	const BVHConfig &m_config;
	float m_min_overlap_area;

	struct Node {
		AABB aabb;
		Node *left{}, *right{};
		uint32_t tri_idx{}, node_idx{};
	} m_root{};
	AtomicAllocator<Node, kAtomicAllocatorChunk> m_node_pool;

	std::vector<LocalAllocator<Node, kAtomicAllocatorChunk>> m_thread_node_allocators;

	std::atomic_uint32_t m_node_count{1};
	inline Node *new_node(LocalAllocator<Node, kAtomicAllocatorChunk> *p_allocator) {
		auto ret = p_allocator->Alloc();
		ret->node_idx = m_node_count++;
		return ret;
	}

	struct Reference {
		AABB aabb;
		uint32_t tri_idx{};
	};

	template <uint32_t DIM> inline static bool reference_cmp(const Reference &l, const Reference &r);
	template <uint32_t DIM, typename Iter> inline static void sort_references(Iter first_ref, Iter last_ref);
	template <typename Iter> inline static void sort_references(Iter first_ref, Iter last_ref, uint32_t dim);
	inline std::tuple<Reference, Reference> split_reference(const Reference &ref, uint32_t dim, float pos) const;

	// A simple thread pool with 1 thread and 1 task
	class ThreadUnit {
	private:
		std::optional<std::function<void()>> m_task;
		std::condition_variable m_task_cv;
		std::mutex m_task_mutex;
		std::thread m_thread;
		std::atomic_bool m_run;

		inline void worker_func() {
			for (std::function<void()> task;;) {
				{
					std::unique_lock<std::mutex> lock{m_task_mutex};
					m_task_cv.wait(lock,
					               [this] { return m_task.has_value() || !m_run.load(std::memory_order_acquire); });
					if (!m_run.load(std::memory_order_acquire) && !m_task.has_value())
						return;
					task = std::move(m_task.value());
					m_task.reset();
				}
				task();
			}
		}

	public:
		inline ThreadUnit() : m_run{true} { m_thread = std::thread(&ThreadUnit::worker_func, this); }
		inline ~ThreadUnit() {
			m_run.store(false, std::memory_order_release);
			m_task_cv.notify_one();
			m_thread.join();
		}
		template <class F, class... Args, typename R = std::result_of_t<F(Args...)>>
		inline std::future<R> Push(F &&f, Args &&...args) {
			auto task =
			    std::make_shared<std::packaged_task<R()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
			std::future<R> res = task->get_future();
			{
				std::scoped_lock<std::mutex> lock{m_task_mutex};
				m_task = [task]() { (*task)(); };
			}
			m_task_cv.notify_one();
			return res;
		}
	};
	std::unique_ptr<ThreadUnit[]> m_thread_group;

	class Task {
	private:
		ParallelSBVHBuilder *m_p_builder{};
		Node *m_node{};
		std::vector<Reference> m_references;
		uint32_t m_depth{}, m_thread{}, m_thread_count{};

		struct ObjectSplit {
			AABB left_aabb, right_aabb;
			uint32_t dim{};
			float pos{}, sah{FLT_MAX}, bin_width{};
		};
		struct ObjectBin {
			AABB aabb{};
			uint32_t cnt{};
		};
		struct SpatialSplit {
			uint32_t dim{};
			float pos{}, sah{FLT_MAX};
		};
		struct SpatialBin {
			AABB aabb{};
			uint32_t in{}, out{};
		};

		template <uint32_t DIM> inline void _find_spatial_split_dim(SpatialSplit *p_ss);
		inline void _find_spatial_split_parallel(SpatialSplit *p_ss);
		inline SpatialSplit find_spatial_split();
		inline std::tuple<Task, Task> perform_spatial_split(const SpatialSplit &ss);

		template <uint32_t DIM> inline void _find_object_split_sweep_dim(ObjectSplit *p_os);
		template <uint32_t DIM> inline void _find_object_split_binned_dim(ObjectSplit *p_os);
		inline void _find_object_split_binned_parallel(ObjectSplit *p_os);
		inline ObjectSplit find_object_split();
		inline std::tuple<Task, Task> perform_object_split(const ObjectSplit &os);

		inline std::tuple<uint32_t, uint32_t> get_child_thread_counts(uint32_t left_ref_count,
		                                                              uint32_t right_ref_count) const;

		inline void perform_leaf() { m_node->tri_idx = m_references.front().tri_idx; }

		inline ThreadUnit &get_thread_unit(uint32_t idx = 0) const {
			return m_p_builder->m_thread_group[m_thread + idx - 1];
		}

		inline const moodycamel::ProducerToken &get_queue_producer_token() const {
			return m_p_builder->m_producer_tokens[m_thread];
		}
		inline moodycamel::ConsumerToken &get_queue_consumer_token() const {
			return m_p_builder->m_consumer_tokens[m_thread];
		}
		inline Node *new_node() const {
			return m_p_builder->new_node(&m_p_builder->m_thread_node_allocators[m_thread]);
		}
		inline void assign_to_thread(uint32_t thread) {
			m_thread = thread;
			m_thread_count = 0;
		}

	public:
		inline Task() = default;
		inline Task(ParallelSBVHBuilder *p_builder, Node *node, std::vector<Reference> &&references, uint32_t depth,
		            uint32_t thread_begin, uint32_t thread_count)
		    : m_p_builder{p_builder}, m_node{node}, m_references{std::move(references)}, m_depth{depth},
		      m_thread(thread_begin), m_thread_count{thread_count} {}
		Task(const Task &r) = delete;
		Task &operator=(const Task &r) = delete;

		Task(Task &&r) = default;
		Task &operator=(Task &&r) = default;

		inline static bool PairEmpty(const std::tuple<Task, Task> &p) { return std::get<0>(p).Empty(); }
		inline bool Empty() const { return !m_node; }
		std::tuple<Task, Task> Run();
		void BlockRun();
		std::future<void> AsyncRun();
		void LocalRun();
	};

	// Task queue
	std::atomic_uint32_t m_task_count{};
	moodycamel::ConcurrentQueue<Task> m_task_queue;
	std::vector<moodycamel::ConsumerToken> m_consumer_tokens;
	std::vector<moodycamel::ProducerToken> m_producer_tokens;

	Task get_root_task();
	// void local_run_task(Task &&task);
	uint32_t fetch_result(Node *cur, std::vector<BinaryBVH::Node> *p_nodes, uint32_t *p_leaf_count);

public:
	ParallelSBVHBuilder(const BVHConfig &config, const Scene &scene)
	    : kThreadCount(std::max(1u, std::thread::hardware_concurrency())), m_scene(scene),
	      m_config(config), m_min_overlap_area{scene.GetAABB().GetArea() * 1e-5f} {}
	void Run();
	void FetchResult(std::vector<BinaryBVH::Node> *p_nodes, uint32_t *p_leaf_count);
};

#endif
