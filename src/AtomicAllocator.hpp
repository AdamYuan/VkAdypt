//
// Created by adamyuan on 7/21/22.
//

#ifndef ADYPT_ATOMICALLOCATOR_HPP
#define ADYPT_ATOMICALLOCATOR_HPP

#include <atomic>
#include <cinttypes>
#include <cstdlib>
#include <spdlog/spdlog.h>
#include <vector>

template <class T> class AtomicAllocator {
private:
	struct Node {
		T chunk[1u << 16u];
	};
	std::atomic_uint32_t m_head{0};
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

template <class T> class AtomicBlockAllocator {
private:
	std::atomic_uint32_t m_head{0};
	T *m_chunks[1u << 16u];

public:
	inline AtomicBlockAllocator(const AtomicBlockAllocator &r) = delete;
	inline AtomicBlockAllocator &operator=(const AtomicBlockAllocator &r) = delete;
	inline AtomicBlockAllocator(AtomicBlockAllocator &&r) noexcept = default;
	inline AtomicBlockAllocator &operator=(AtomicBlockAllocator &&r) noexcept = default;

	inline AtomicBlockAllocator() = default;
	inline ~AtomicBlockAllocator() {
		uint32_t head = m_head.load();
		for (uint32_t i = 0; i < head; ++i)
			delete[] m_chunks[i];
	}
	inline T *AllocChunk(uint32_t count) {
		uint32_t id = m_head++;
		if (id > 0xffffu)
			return nullptr;
		m_chunks[id] = new T[count];
		return m_chunks[id];
	}
};

template <class T> class LocalBlockAllocator {
private:
	AtomicBlockAllocator<T> &m_allocator_ref;
	T *m_chunk{};
	uint32_t m_chunk_size{}, m_counter{};

	inline T *new_chunk_alloc(uint32_t count) {
		m_chunk_size = std::max(count, 1u << 16u);
		m_counter = count;
		m_chunk = m_allocator_ref.AllocChunk(m_chunk_size);
		return m_chunk;
	}

public:
	inline LocalBlockAllocator(const LocalBlockAllocator &r) = delete;
	inline LocalBlockAllocator &operator=(const LocalBlockAllocator &r) = delete;
	inline LocalBlockAllocator(LocalBlockAllocator &&r) noexcept = default;
	inline LocalBlockAllocator &operator=(LocalBlockAllocator &&r) noexcept = default;

	inline explicit LocalBlockAllocator(AtomicBlockAllocator<T> &allocator) : m_allocator_ref{allocator} {}
	inline explicit LocalBlockAllocator(AtomicBlockAllocator<T> &allocator, uint32_t initial_count)
	    : m_allocator_ref{allocator} {
		m_chunk_size = std::max(initial_count, 1u << 16u);
		m_counter = 0;
		m_chunk = m_allocator_ref.AllocChunk(m_chunk_size);
	}
	inline T *Alloc(uint32_t count) {
		if (m_chunk && m_counter + count < m_chunk_size) {
			uint32_t idx = m_counter;
			m_counter += count;
			return m_chunk + idx;
		}
		return new_chunk_alloc(count);
	}
};

#endif // ADYPT_ATOMICALLOCATOR_HPP
