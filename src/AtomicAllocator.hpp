//
// Created by adamyuan on 7/21/22.
//

#ifndef ADYPT_ATOMICALLOCATOR_HPP
#define ADYPT_ATOMICALLOCATOR_HPP

#include <atomic>
#include <cinttypes>

template <class T> class AtomicAllocator {
private:
	struct Node {
		T chunk[1u << 16u];
	};
	std::atomic_uint16_t m_head{0};
	Node *m_chunks[1u << 16u];

public:
	inline AtomicAllocator(const AtomicAllocator &r) = delete;
	inline AtomicAllocator &operator=(const AtomicAllocator &r) = delete;
	inline AtomicAllocator(AtomicAllocator &&r) noexcept = default;
	inline AtomicAllocator &operator=(AtomicAllocator &&r) noexcept = default;

	inline AtomicAllocator() = default;
	inline ~AtomicAllocator() {
		uint32_t head = m_head.load();
		for (uint32_t i = 0; i < head; ++i)
			delete m_chunks[i];
	}
	inline uint32_t AllocChunk() {
		uint32_t id = m_head++;
		m_chunks[id] = new Node{};
		return id << 16u;
	}
	inline uint32_t GetRange() const { return m_head.load() << 16u; }
	T &operator[](uint32_t id) { return m_chunks[id >> 16u]->chunk[id & 0xffffu]; }
	const T &operator[](uint32_t id) const { return m_chunks[id >> 16u]->chunk[id & 0xffffu]; }
};

template <class T> class LocalAllocator {
private:
	AtomicAllocator<T> &m_allocator_ref;
	uint32_t m_chunk, m_counter{};

public:
	inline LocalAllocator(const LocalAllocator &r) = delete;
	inline LocalAllocator &operator=(const LocalAllocator &r) = delete;
	inline LocalAllocator(LocalAllocator &&r) noexcept = default;
	inline LocalAllocator &operator=(LocalAllocator &&r) noexcept = default;

	inline explicit LocalAllocator(AtomicAllocator<T> &allocator)
	    : m_allocator_ref{allocator}, m_chunk{allocator.AllocChunk()} {}
	inline uint32_t Alloc() {
		if (m_counter <= 0xffffu)
			return m_chunk | (m_counter++);
		else {
			m_counter = 1;
			return (m_chunk = m_allocator_ref.AllocChunk());
		}
	}
};

#endif // ADYPT_ATOMICALLOCATOR_HPP
