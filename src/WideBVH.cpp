#include "WideBVH.hpp"
#include <cstring>
#include <fstream>

bool WideBVH::SaveToFile(const char *filename) {
	std::ofstream os{filename, std::ios::binary};
	if (!os.is_open())
		return false;

	os.write(kVersionStr, strlen(kVersionStr) + 1);
	os.write((char *)&m_config, sizeof(BVHConfig));
	auto tri_indeices_size = (uint32_t)m_tri_indices.size();
	os.write((char *)&tri_indeices_size, sizeof(uint32_t));
	os.write((char *)m_tri_indices.data(), m_tri_indices.size() * sizeof(int32_t));
	os.write((char *)m_nodes.data(), m_nodes.size() * sizeof(Node));

	return true;
}

/*std::shared_ptr<WideBVH> WideBVH::CreateFromFile(const char *filename, const BVHConfig &expected_config) {
    std::shared_ptr<WideBVH> ret = std::make_shared<WideBVH>();
    ret->m_config = expected_config;

    std::ifstream is{filename, std::ios::binary};
    if (!is.is_open())
        return nullptr;

    std::vector<char> buffer{std::istreambuf_iterator<char>(is), std::istreambuf_iterator<char>()};
    char *ptr = buffer.data(), *end = buffer.data() + buffer.size();

    // compare cwbvh version
    if (strcmp(ptr, kVersionStr) != 0)
        return nullptr;

    ptr += strlen(kVersionStr) + 1;

    auto config = *(BVHConfig *)ptr;
    ptr += sizeof(BVHConfig);

    if (config.m_node_sah != expected_config.m_node_sah || config.m_triangle_sah != expected_config.m_triangle_sah ||
        config.m_max_spatial_depth != expected_config.m_max_spatial_depth)
        return nullptr;

    uint32_t tri_indices_size = *(uint32_t *)ptr;
    ptr += sizeof(uint32_t);

    // read triangle indices
    for (int i = 0; i < tri_indices_size && ptr < end; ++i) {
        ret->m_tri_indices.push_back(*(int32_t *)ptr);
        ptr += sizeof(int32_t);
    }

    // read node
    while (ptr < end) {
        ret->m_nodes.push_back(*(WideBVHNode *)ptr);
        ptr += sizeof(WideBVHNode);
    }

    return ret;
}*/
