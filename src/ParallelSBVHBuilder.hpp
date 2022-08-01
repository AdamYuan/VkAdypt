#ifndef ADYPT_PARALLELSBVHBUILDER_HPP
#define ADYPT_PARALLELSBVHBUILDER_HPP

#include "AtomicAllocator.hpp"
#include "AtomicBinaryBVH.hpp"
#include <atomic>
#include <cfloat>
#include <concurrentqueue.h>
#include <condition_variable>
#include <functional>
#include <future>
#include <optional>
#include <utility>

class ParallelSBVHBuilder {
public:
	using BVHType = AtomicBinaryBVH;

private:
	const uint32_t kThreadCount;
	static constexpr uint32_t kSpatialBinNum = 32, kObjectBinNum = 32;
	static constexpr uint32_t kParallelForBlockSize = 64;
	static constexpr uint32_t kLocalRunThreshold = 512;
	inline static constexpr uint32_t GetReferenceBlockSize(uint32_t ref_cnt) { return ref_cnt * 3 / 2; }

	AtomicBinaryBVH &m_bvh;
	const Scene &m_scene;
	const BVHConfig &m_config;
	float m_min_overlap_area;

	AtomicAllocator<AtomicBinaryBVH::Node> &m_node_pool;
	std::vector<LocalAllocator<AtomicBinaryBVH::Node>> m_thread_node_allocators;

	std::atomic_uint32_t m_leaf_count{0};

	struct Reference {
		AABB aabb;
		uint32_t tri_idx{};
	};
	AtomicAllocator<Reference> m_reference_pool;
	std::vector<LocalAllocator<Reference>> m_thread_reference_allocators;

	template <uint32_t DIM, typename Iter> inline void sort_references(Iter first_ref, Iter last_ref);
	template <typename Iter> inline void sort_references(Iter first_ref, Iter last_ref, uint32_t dim);
	inline std::tuple<Reference, Reference> split_reference(const Reference &ref, uint32_t dim, float pos) const;

	AtomicBlockAllocator<uint32_t> m_reference_block_pool;
	std::vector<LocalBlockAllocator<uint32_t>> m_thread_reference_block_allocators;
	std::tuple<uint32_t *, uint32_t *, uint32_t> alloc_reference_block(LocalBlockAllocator<uint32_t> *p_block_allocator,
	                                                                   uint32_t origin_ref_cnt) {
		auto single_size = GetReferenceBlockSize(origin_ref_cnt);
		uint32_t *begin = p_block_allocator->Alloc(single_size << 1u);
		if (begin == nullptr)
			return {nullptr, nullptr, 0};
		return {begin, begin + single_size, single_size};
	}

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
	public:
		enum ReferenceAlignment { kAlignLeft = 0, kAlignRight = 1 };

	private:
		ParallelSBVHBuilder *m_p_builder{};
		ReferenceAlignment m_reference_alignment{kAlignLeft};
		uint32_t m_reference_count{}, m_reference_block_size;
		uint32_t *m_reference_block{}, *m_tmp_reference_block{};

		uint32_t m_node_index{};
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
			uint32_t dim{}, ref_cnt{};
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
		inline std::tuple<Task, Task> _perform_spatial_split_inplace(const SpatialSplit &ss);
		inline std::tuple<Task, Task> _perform_spatial_split_tmp(const SpatialSplit &ss, uint32_t *ref_block,
		                                                         uint32_t *tmp_ref_block, uint32_t ref_block_size);
		// inline std::tuple<Task, Task> _perform_spatial_split_parallel(const SpatialSplit &ss);

		template <uint32_t DIM> inline void _find_object_split_sweep_dim(ObjectSplit *p_os);
		template <uint32_t DIM> inline void _find_object_split_binned_dim(ObjectSplit *p_os);
		inline void _find_object_split_binned_parallel(ObjectSplit *p_os);
		inline ObjectSplit find_object_split();
		inline std::tuple<Task, Task> perform_object_split(const ObjectSplit &os);
		inline std::tuple<Task, Task> _perform_object_split_inplace(const ObjectSplit &os);
		inline std::tuple<AtomicBinaryBVH::Node &, AtomicBinaryBVH::Node &> maintain_child_nodes() {
			auto &node = access_node(m_node_index);
			if (!node.left)
				node.left = new_node();
			if (!node.right)
				node.right = new_node();
			return {access_node(node.left), access_node(node.right)};
		}
		// inline std::tuple<Task, Task> _perform_object_split_parallel(const ObjectSplit &os);

		inline std::tuple<Task, Task> perform_default_split();

		inline uint32_t split_thread(uint32_t left_ref_count, uint32_t right_ref_count) const;
		static inline uint32_t split_reference_block(uint32_t left_ref_count, uint32_t right_ref_count,
		                                             uint32_t ref_block_size);
		inline std::tuple<Task, Task> split_task_with_block(uint32_t left_ref_count, uint32_t right_ref_count,
		                                                    uint32_t ref_block_size, uint32_t *ref_block,
		                                                    uint32_t *tmp_ref_block) const;
		inline std::tuple<Task, Task> split_task(uint32_t left_ref_count, uint32_t right_ref_count,
		                                         bool swap_to_tmp) const;

		inline AtomicBinaryBVH::Node &access_node(uint32_t node_idx) const {
			return m_p_builder->m_node_pool[node_idx];
		}
		inline Reference &access_reference(uint32_t ref_idx) const { return m_p_builder->m_reference_pool[ref_idx]; }
		inline uint32_t *get_reference_begin() const {
			return m_reference_alignment == kAlignLeft ? m_reference_block
			                                           : m_reference_block + m_reference_block_size - m_reference_count;
		}

		inline void make_leaf() {
			auto &node = access_node(m_node_index);
			node.tri_idx = access_reference(*get_reference_begin()).tri_idx;
			++m_p_builder->m_leaf_count;
		}

		inline ThreadUnit &get_thread_unit(uint32_t idx = 0) const {
			return m_p_builder->m_thread_group[m_thread + idx - 1];
		}

		inline const moodycamel::ProducerToken &get_queue_producer_token() const {
			return m_p_builder->m_producer_tokens[m_thread];
		}
		inline moodycamel::ConsumerToken &get_queue_consumer_token() const {
			return m_p_builder->m_consumer_tokens[m_thread];
		}
		inline uint32_t new_node() const { return m_p_builder->m_thread_node_allocators[m_thread].Alloc(); }
		inline uint32_t new_reference(uint32_t idx = 0) const {
			return m_p_builder->m_thread_reference_allocators[m_thread + idx].Alloc();
		}
		inline std::tuple<uint32_t *, uint32_t *, uint32_t> new_reference_block(uint32_t origin_ref_cnt) {
			return m_p_builder->alloc_reference_block(
			    m_p_builder->m_thread_reference_block_allocators.data() + m_thread, origin_ref_cnt);
		}

	public:
		inline Task() = default;
		inline Task(ParallelSBVHBuilder *p_builder, uint32_t node_index, ReferenceAlignment reference_alignment,
		            uint32_t reference_count, uint32_t reference_block_size, uint32_t *reference_block,
		            uint32_t *tmp_reference_block, uint32_t depth, uint32_t thread_begin, uint32_t thread_count)
		    : m_p_builder{p_builder}, m_node_index{node_index}, m_reference_alignment{reference_alignment},
		      m_reference_count{reference_count}, m_reference_block_size{reference_block_size},
		      m_reference_block{reference_block}, m_tmp_reference_block{tmp_reference_block}, m_depth{depth},
		      m_thread(thread_begin), m_thread_count{thread_count} {}
		Task(const Task &r) = delete;
		Task &operator=(const Task &r) = delete;

		Task(Task &&r) = default;
		Task &operator=(Task &&r) = default;

		inline static bool PairEmpty(const std::tuple<Task, Task> &p) { return std::get<0>(p).Empty(); }
		inline bool Empty() const { return !m_node_index; }
		std::tuple<Task, Task> Run();
		void BlockRun();
		std::future<void> AsyncRun();
		void LocalRun();
		inline void AssignToThread(uint32_t thread) {
			m_thread = thread;
			m_thread_count = 0;
		}
	};

	// Task queue
	std::atomic_uint32_t m_task_count{};
	moodycamel::ConcurrentQueue<Task> m_task_queue;
	std::vector<moodycamel::ConsumerToken> m_consumer_tokens;
	std::vector<moodycamel::ProducerToken> m_producer_tokens;

	Task make_root_task();

public:
	explicit ParallelSBVHBuilder(AtomicBinaryBVH *p_bvh)
	    : kThreadCount(std::max(1u, std::thread::hardware_concurrency())), m_bvh{*p_bvh},
	      m_node_pool{p_bvh->m_node_pool}, m_scene(*p_bvh->GetScenePtr()),
	      m_config(p_bvh->GetConfig()), m_min_overlap_area{p_bvh->GetScenePtr()->GetAABB().GetArea() * 1e-5f} {}
	void Run();
};

#endif
