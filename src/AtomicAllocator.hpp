//
// Created by adamyuan on 7/21/22.
//

#ifndef ADYPT_ATOMICALLOCATOR_HPP
#define ADYPT_ATOMICALLOCATOR_HPP

#include <atomic>
#include <cinttypes>

template <class T, uint32_t ChunkSize> class AtomicAllocator {
private:
	struct Node {
		Node *next{};
		T chunk[ChunkSize]{};
	};
	std::atomic<Node *> m_list{};

public:
	inline AtomicAllocator(const AtomicAllocator &r) = delete;
	inline AtomicAllocator &operator=(const AtomicAllocator &r) = delete;
	inline AtomicAllocator(AtomicAllocator &&r) noexcept = default;
	inline AtomicAllocator &operator=(AtomicAllocator &&r) noexcept = default;

	inline AtomicAllocator() = default;
	inline ~AtomicAllocator() {
		Node *cur = m_list.load(std::memory_order_acquire);
		while (cur) {
			Node *next = cur->next;
			delete cur;
			cur = next;
		}
	}
	inline T *AllocChunk() {
		auto node = new Node{};
		node->next = m_list.load(std::memory_order_relaxed);
		while (!m_list.compare_exchange_weak(node->next, node, std::memory_order_release, std::memory_order_relaxed))
			;
		return node->chunk;
	}
};

template <class T, uint32_t ChunkSize> class LocalAllocator {
private:
	AtomicAllocator<T, ChunkSize> &m_allocator_ref;
	T *m_p_chunk;
	uint32_t m_counter{};

public:
	inline LocalAllocator(const LocalAllocator &r) = delete;
	inline LocalAllocator &operator=(const LocalAllocator &r) = delete;
	inline LocalAllocator(LocalAllocator &&r) noexcept = default;
	inline LocalAllocator &operator=(LocalAllocator &&r) noexcept = default;

	inline explicit LocalAllocator(AtomicAllocator<T, ChunkSize> &allocator)
	    : m_allocator_ref{allocator}, m_p_chunk{allocator.AllocChunk()} {}
	inline T *Alloc() {
		if (m_counter < ChunkSize)
			return m_p_chunk + (m_counter++);
		else {
			m_counter = 1;
			return (m_p_chunk = m_allocator_ref.AllocChunk());
		}
	}
};

#endif // ADYPT_ATOMICALLOCATOR_HPP
