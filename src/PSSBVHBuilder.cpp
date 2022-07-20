#include "BVHConfig.hpp"
#include "FlatSBVH.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <spdlog/spdlog.h>

template <class T> class ChunkBuffer {
private:
	std::vector<T> m_data;
	std::atomic_uint32_t m_counter;

public:
	static constexpr uint32_t kChunkSize = 64;
	inline ChunkBuffer(uint32_t size) {
		uint32_t chunks = size / kChunkSize + 1;
		m_data.resize(chunks * kChunkSize);
		m_counter = 0;
	}
	inline uint32_t AllocChunk() { return (m_counter++) * kChunkSize; } // return chunk base index
	inline const std::vector<T> &GetData() const { return m_data; }
	inline std::vector<T> &GetData() { return m_data; }
	inline const T &operator[](uint32_t i) const { return m_data[i]; }
	inline T &operator[](uint32_t i) { return m_data[i]; }
};

template <class T> class AtomicAllocator {
private:
	ChunkBuffer<T> &m_chunk_buffer_ref;
	uint32_t m_chunk_base, m_local_counter;

public:
	inline AtomicAllocator(ChunkBuffer<T> &chunk_buffer) : m_chunk_buffer_ref{chunk_buffer} {
		m_chunk_base = m_chunk_buffer_ref.AllocChunk();
		m_local_counter = 0;
	}
	inline uint32_t Alloc() {
		if (m_local_counter < ChunkBuffer<T>::kChunkSize)
			return m_chunk_base + (m_local_counter++);
		else {
			m_local_counter = 0;
			return m_chunk_base = m_chunk_buffer_ref.AllocChunk();
		}
	}
};

class PSSBVHBuilder {
private:
	static constexpr uint32_t kSpatialBinNum = 32;

	FlatSBVH *m_bvh;
	const Scene &m_scene;
	const BVHConfig &m_config;

public:
	PSSBVHBuilder(FlatSBVH *bvh, const Scene &scene) : m_scene(scene), m_bvh(bvh), m_config(bvh->m_config) {
		m_bvh->m_leaves_cnt = 0;
		m_bvh->m_nodes.clear();
	}
	void Run() {

	}
};

std::shared_ptr<FlatSBVH> FlatSBVH::BuildParalleled(const BVHConfig &config, const std::shared_ptr<Scene> &scene) {
	std::shared_ptr<FlatSBVH> ret = std::make_shared<FlatSBVH>();
	ret->m_config = config;
	ret->m_scene_ptr = scene;
	PSSBVHBuilder builder{ret.get(), *scene};
	builder.Run();
	return ret;
}
