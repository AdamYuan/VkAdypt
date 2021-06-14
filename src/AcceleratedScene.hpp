#ifndef ACCELERATED_SCENE_HPP
#define ACCELERATED_SCENE_HPP

#include "WideBVH.hpp"

#include <unordered_map>

#include <myvk/Buffer.hpp>
#include <myvk/DescriptorSet.hpp>
#include <myvk/Image.hpp>
#include <myvk/ImageView.hpp>
#include <myvk/Sampler.hpp>

class AcceleratedScene {
private:
	struct Material {
		uint32_t m_dtex;
		float m_dr, m_dg, m_db;
		uint32_t m_etex;
		float m_er, m_eg, m_eb;
		uint32_t m_stex;
		float m_sr, m_sg, m_sb;
		uint32_t m_illum;
		float m_shininess, m_dissolve, m_refraction_index;
	};
	struct Texture {
		std::shared_ptr<myvk::Image> m_image;
		std::shared_ptr<myvk::ImageView> m_image_view;
	};
	std::vector<Texture> m_textures;
	std::shared_ptr<myvk::Sampler> m_sampler;

	std::shared_ptr<myvk::Buffer> m_triangles_buffer, m_tri_materials_buffer;
	std::shared_ptr<myvk::Buffer> m_bvh_nodes_buffer, m_bvh_tri_indices_buffer, m_bvh_tri_matrices_buffer;

	std::shared_ptr<myvk::DescriptorPool> m_descriptor_pool;
	std::shared_ptr<myvk::DescriptorSetLayout> m_descriptor_set_layout;
	std::shared_ptr<myvk::DescriptorSet> m_descriptor_set;

	std::vector<glm::vec4> generate_bvh_tri_matrices(const std::shared_ptr<WideBVH> &widebvh);
	std::vector<Material> generate_tri_materials(const std::shared_ptr<Scene> &scene,
	                                             std::unordered_map<std::string, uint32_t> *texture_name_map);

	void load_textures(const std::shared_ptr<myvk::Queue> &graphics_queue, const std::string &base_dir,
	                   const std::unordered_map<std::string, uint32_t> &texture_name_map);

	void process_texture_errors(std::vector<Material> *tri_materials);

	void create_bvh_buffers(const std::shared_ptr<myvk::Queue> &graphics_queue,
	                        const std::shared_ptr<WideBVH> &widebvh);
	void create_triangle_buffers(const std::shared_ptr<myvk::Queue> &graphics_queue,
	                             const std::shared_ptr<Scene> &scene);
	void create_descriptors(const std::shared_ptr<myvk::Device> &device);

public:
	static std::shared_ptr<AcceleratedScene> Create(const std::shared_ptr<myvk::Queue> &graphics_queue,
	                                                const std::shared_ptr<WideBVH> &widebvh);

	uint32_t GetTextureCount() const { return m_textures.size(); }
	const std::shared_ptr<myvk::DescriptorSet> &GetDescriptorSet() const { return m_descriptor_set; }
	const std::shared_ptr<myvk::DescriptorSetLayout> &GetDescriptorSetLayout() const { return m_descriptor_set_layout; }
};

#endif
