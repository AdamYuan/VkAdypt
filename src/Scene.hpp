#ifndef PRIMITIVE_HPP
#define PRIMITIVE_HPP

#include "Shape.hpp"
#include <memory>
#include <tiny_obj_loader.h>
#include <vector>

// load some basic components
struct Scene {
private:
	std::vector<Triangle> m_triangles;
	AABB m_aabb{};
	std::vector<tinyobj::material_t> m_materials;
	std::string m_base_dir;

	void extract_shapes(const tinyobj::attrib_t &attrib, const std::vector<tinyobj::shape_t> &shapes);
	void normalize();

public:
	static std::shared_ptr<Scene> CreateFromFile(const char *filename);

	const std::vector<Triangle> &GetTriangles() const { return m_triangles; }
	const std::vector<tinyobj::material_t> &GetTinyobjMaterials() const { return m_materials; }
	const std::string &GetBasePath() const { return m_base_dir; };
	const AABB &GetAABB() const { return m_aabb; }
};

#endif
