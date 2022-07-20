#include "AcceleratedScene.hpp"

#include <myvk/CommandBuffer.hpp>
#include <myvk/ObjectTracker.hpp>
#include <spdlog/spdlog.h>
#include <stb_image.h>

#include <atomic>
#include <future>
#include <thread>

std::shared_ptr<AcceleratedScene> AcceleratedScene::Create(const std::shared_ptr<myvk::Queue> &graphics_queue,
                                                           const std::shared_ptr<WideBVH> &widebvh) {
	std::shared_ptr<AcceleratedScene> ret = std::make_shared<AcceleratedScene>();

	ret->create_triangle_buffers(graphics_queue, widebvh->GetScenePtr());
	ret->create_bvh_buffers(graphics_queue, widebvh);
	ret->create_descriptors(graphics_queue->GetDevicePtr());

	return ret;
}

std::vector<glm::vec4> AcceleratedScene::generate_bvh_tri_matrices(const std::shared_ptr<WideBVH> &widebvh) {
	std::vector<glm::vec4> matrices;
	matrices.reserve(widebvh->GetTriIndices().size() * 3u);
	for (uint32_t t : widebvh->GetTriIndices()) {
		const Triangle &tri = widebvh->GetScenePtr()->GetTriangles()[t];
		const glm::vec3 &v0 = tri.m_positions[0], &v1 = tri.m_positions[1], &v2 = tri.m_positions[2];
		glm::vec4 c0{v0 - v2, 0.0f};
		glm::vec4 c1{v1 - v2, 0.0f};
		glm::vec4 c2{glm::cross(v0 - v2, v1 - v2), 0.0f};
		glm::vec4 c3{v2, 1.0f};
		glm::mat4 mtx{c0.x, c1.x, c2.x, c3.x, c0.y, c1.y, c2.y, c3.y, c0.z, c1.z, c2.z, c3.z, c0.w, c1.w, c2.w, c3.w};
		mtx = glm::inverse(mtx);
		matrices.emplace_back(mtx[2][0], mtx[2][1], mtx[2][2], -mtx[2][3]);
		matrices.emplace_back(mtx[0][0], mtx[0][1], mtx[0][2], mtx[0][3]);
		matrices.emplace_back(mtx[1][0], mtx[1][1], mtx[1][2], mtx[1][3]);
	}
	return matrices;
}

std::vector<AcceleratedScene::Material>
AcceleratedScene::generate_tri_materials(const std::shared_ptr<Scene> &scene,
                                         std::unordered_map<std::string, uint32_t> *texture_name_map) {
	texture_name_map->clear();

	auto alloc_texture_id = [&texture_name_map](const std::string &str) -> void {
		if (!str.empty() && texture_name_map->find(str) == texture_name_map->end()) {
			uint32_t idx = texture_name_map->size();
			(*texture_name_map)[str] = idx;
		}
	};
	auto get_texture_id = [&texture_name_map](const std::string &str) -> uint32_t {
		if (str.empty())
			return UINT32_MAX;
		return texture_name_map->at(str);
	};

	std::vector<Material> materials;
	materials.reserve(scene->GetTinyobjMaterials().size());

	for (const auto &t : scene->GetTinyobjMaterials()) {
		materials.emplace_back();
		auto &mat = materials.back();

		alloc_texture_id(t.diffuse_texname);
		mat.m_dr = t.diffuse[0];
		mat.m_dg = t.diffuse[1];
		mat.m_db = t.diffuse[2];

		mat.m_etex = UINT32_MAX;
		mat.m_er = t.emission[0];
		mat.m_eg = t.emission[1];
		mat.m_eb = t.emission[2];

		mat.m_stex = UINT32_MAX;
		mat.m_sr = t.specular[0];
		mat.m_sg = t.specular[1];
		mat.m_sb = t.specular[2];

		mat.m_illum = t.illum;
		mat.m_shininess = t.shininess;
		mat.m_dissolve = t.dissolve;
		mat.m_refraction_index = t.ior;
	}

	for (uint32_t i = 0; i < materials.size(); ++i)
		materials[i].m_dtex = get_texture_id(scene->GetTinyobjMaterials()[i].diffuse_texname);

	return materials;
}

void AcceleratedScene::load_textures(const std::shared_ptr<myvk::Queue> &graphics_queue, const std::string &base_dir,
                                     const std::unordered_map<std::string, uint32_t> &texture_name_map) {
	m_textures.clear();
	if (texture_name_map.empty())
		return;

	// create sampler
	m_sampler = myvk::Sampler::Create(graphics_queue->GetDevicePtr(), VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT);

	// process texture filenames
	std::vector<std::string> texture_filenames(texture_name_map.size());
	for (auto &i : texture_name_map)
		texture_filenames[i.second] = base_dir + i.first;

	m_textures.resize(texture_filenames.size());

	// multi-threaded texture loading
	const std::shared_ptr<myvk::Device> &device = graphics_queue->GetDevicePtr();
	unsigned cores = std::thread::hardware_concurrency();
	std::vector<std::future<void>> future_vector;
	future_vector.reserve(cores);
	std::atomic_uint32_t texture_id{0};
	while (cores--) {
		future_vector.push_back(std::async([&]() -> void {
			std::shared_ptr<myvk::CommandPool> command_pool = myvk::CommandPool::Create(graphics_queue);
			myvk::ObjectTracker tracker;
			while (true) {
				uint32_t i = texture_id++;
				if (i >= texture_filenames.size())
					break;

				// spdlog::info("{}/{} {}", i, texture_filenames.size(), texture_filenames[i]);

				// Load texture data from file
				int width, height, channels;
				stbi_uc *data = stbi_load(texture_filenames[i].c_str(), &width, &height, &channels, 4);
				if (data == nullptr) {
					spdlog::error("Unable to load texture {}, {}", texture_filenames[i].c_str(), stbi_failure_reason());
					continue;
				}
				uint32_t data_size = width * height * 4;
				VkExtent2D extent = {(uint32_t)width, (uint32_t)height};
				// Create staging buffer
				std::shared_ptr<myvk::Buffer> staging_buffer = myvk::Buffer::CreateStaging(device, data_size);
				staging_buffer->UpdateData(data, data + data_size);
				// Free texture data
				stbi_image_free(data);

				Texture &texture = m_textures[i];
				// Create image
				texture.m_image =
				    myvk::Image::CreateTexture2D(device, extent, 1, VK_FORMAT_R8G8B8A8_SRGB,
				                                 VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT);
				// Create ImageView and Sampler
				texture.m_image_view = myvk::ImageView::Create(texture.m_image, VK_IMAGE_VIEW_TYPE_2D);

				// Copy buffer to image and generate mipmap
				std::shared_ptr<myvk::Fence> fence = myvk::Fence::Create(device);
				std::shared_ptr<myvk::CommandBuffer> command_buffer = myvk::CommandBuffer::Create(command_pool);
				command_buffer->Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
				VkBufferImageCopy region = {};
				region.bufferOffset = 0;
				region.bufferRowLength = 0;
				region.bufferImageHeight = 0;
				region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
				region.imageSubresource.mipLevel = 0;
				region.imageSubresource.baseArrayLayer = 0;
				region.imageSubresource.layerCount = 1;
				region.imageOffset = {0, 0, 0};
				region.imageExtent = {(uint32_t)width, (uint32_t)height, 1};

				command_buffer->CmdPipelineBarrier(
				    VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, {}, {},
				    texture.m_image->GetDstMemoryBarriers({region}, 0, VK_ACCESS_TRANSFER_WRITE_BIT,
				                                          VK_IMAGE_LAYOUT_UNDEFINED,
				                                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL));
				command_buffer->CmdCopy(staging_buffer, texture.m_image, {region});
				command_buffer->CmdPipelineBarrier(
				    VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, {}, {},
				    texture.m_image->GetDstMemoryBarriers({region}, VK_ACCESS_TRANSFER_WRITE_BIT, 0,
				                                          VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
				                                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL));

				command_buffer->End();
				tracker.Track(fence, {command_buffer, staging_buffer});
				tracker.Update();

				command_buffer->Submit(fence);

				spdlog::info("Texture {} loaded", texture_filenames[i].c_str());
			}
		}));
	}
}

void AcceleratedScene::process_texture_errors(std::vector<AcceleratedScene::Material> *tri_materials) {
	if (m_textures.empty())
		return;
	std::vector<uint32_t> new_index_mapper(m_textures.size()), prefix(m_textures.size());
	// generate prefix
	for (uint32_t i = 0; i < m_textures.size(); ++i)
		prefix[i] = (m_textures[i].m_image != nullptr);
	// sum the prefix
	for (uint32_t i = 0; i < m_textures.size(); ++i) {
		if (prefix[i] == 0)
			new_index_mapper[i] = UINT32_MAX;
		if (i > 0)
			prefix[i] += prefix[i - 1];
		if (new_index_mapper[i] != UINT32_MAX)
			new_index_mapper[i] = prefix[i] - 1;
	}
	// move textures
	for (uint32_t i = 0; i < m_textures.size(); ++i) {
		if (new_index_mapper[i] != UINT32_MAX)
			m_textures[new_index_mapper[i]] = m_textures[i];
	}
	m_textures.resize(prefix.back());
	spdlog::info("{} textures loaded", m_textures.size());

	for (auto &mat : *tri_materials) {
		if (~mat.m_dtex)
			mat.m_dtex = new_index_mapper[mat.m_dtex];
	}
}

void AcceleratedScene::create_triangle_buffers(const std::shared_ptr<myvk::Queue> &graphics_queue,
                                               const std::shared_ptr<Scene> &scene) {
	const std::shared_ptr<myvk::Device> &device = graphics_queue->GetDevicePtr();

	std::shared_ptr<myvk::Buffer> triangles_staging_buffer, tri_materials_staging_buffer;

	{ // create triangles_staging_buffer
		const std::vector<Triangle> &triangles = scene->GetTriangles();
		triangles_staging_buffer = myvk::Buffer::CreateStaging(device, triangles.size() * sizeof(Triangle));
		triangles_staging_buffer->UpdateData(triangles.data(), triangles.data() + triangles.size());
	}

	{ // create tri_materials_staging_buffer
		std::unordered_map<std::string, uint32_t> texture_name_map;
		std::vector<Material> tri_materials = generate_tri_materials(scene, &texture_name_map);
		load_textures(graphics_queue, scene->GetBasePath(), texture_name_map);
		process_texture_errors(&tri_materials);

		tri_materials_staging_buffer = myvk::Buffer::CreateStaging(device, tri_materials.size() * sizeof(Material));
		tri_materials_staging_buffer->UpdateData(tri_materials.data(), tri_materials.data() + tri_materials.size());
	}

	m_triangles_buffer = myvk::Buffer::Create(device, triangles_staging_buffer->GetSize(), VMA_MEMORY_USAGE_GPU_ONLY,
	                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
	m_tri_materials_buffer =
	    myvk::Buffer::Create(device, tri_materials_staging_buffer->GetSize(), VMA_MEMORY_USAGE_GPU_ONLY,
	                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);

	std::shared_ptr<myvk::Fence> fence = myvk::Fence::Create(device);
	std::shared_ptr<myvk::CommandPool> command_pool = myvk::CommandPool::Create(graphics_queue);
	std::shared_ptr<myvk::CommandBuffer> command_buffer = myvk::CommandBuffer::Create(command_pool);
	command_buffer->Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
	command_buffer->CmdCopy(triangles_staging_buffer, m_triangles_buffer,
	                        {{0, 0, triangles_staging_buffer->GetSize()}});
	command_buffer->CmdCopy(tri_materials_staging_buffer, m_tri_materials_buffer,
	                        {{0, 0, tri_materials_staging_buffer->GetSize()}});
	command_buffer->End();
	command_buffer->Submit(fence);

	fence->Wait();

	spdlog::info("triangle buffers generated");
}

void AcceleratedScene::create_bvh_buffers(const std::shared_ptr<myvk::Queue> &graphics_queue,
                                          const std::shared_ptr<WideBVH> &widebvh) {
	const std::shared_ptr<myvk::Device> &device = graphics_queue->GetDevicePtr();

	std::shared_ptr<myvk::Buffer> bvh_nodes_staging_buffer, bvh_tri_indices_staging_buffer,
	    bvh_tri_matrices_staging_buffer;

	{ // create bvh_nodes_staging_buffer
		const std::vector<WideBVH::Node> &nodes = widebvh->GetNodes();
		bvh_nodes_staging_buffer = myvk::Buffer::CreateStaging(device, nodes.size() * sizeof(WideBVH::Node));
		bvh_nodes_staging_buffer->UpdateData(nodes.data(), nodes.data() + nodes.size());
	}

	{ // create bvh_tri_indices_staging_buffer
		const std::vector<uint32_t> &tri_indices = widebvh->GetTriIndices();
		bvh_tri_indices_staging_buffer = myvk::Buffer::CreateStaging(device, tri_indices.size() * sizeof(uint32_t));
		bvh_tri_indices_staging_buffer->UpdateData(tri_indices.data(), tri_indices.data() + tri_indices.size());
	}

	{ // create bvh_tri_matrices_staging_buffer
		std::vector<glm::vec4> tri_matrices = generate_bvh_tri_matrices(widebvh);
		bvh_tri_matrices_staging_buffer = myvk::Buffer::CreateStaging(device, tri_matrices.size() * sizeof(glm::vec4));
		bvh_tri_matrices_staging_buffer->UpdateData(tri_matrices.data(), tri_matrices.data() + tri_matrices.size());
	}

	m_bvh_nodes_buffer = myvk::Buffer::Create(device, bvh_nodes_staging_buffer->GetSize(), VMA_MEMORY_USAGE_GPU_ONLY,
	                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
	m_bvh_tri_indices_buffer =
	    myvk::Buffer::Create(device, bvh_tri_indices_staging_buffer->GetSize(), VMA_MEMORY_USAGE_GPU_ONLY,
	                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
	m_bvh_tri_matrices_buffer =
	    myvk::Buffer::Create(device, bvh_tri_matrices_staging_buffer->GetSize(), VMA_MEMORY_USAGE_GPU_ONLY,
	                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);

	std::shared_ptr<myvk::Fence> fence = myvk::Fence::Create(device);
	std::shared_ptr<myvk::CommandPool> command_pool = myvk::CommandPool::Create(graphics_queue);
	std::shared_ptr<myvk::CommandBuffer> command_buffer = myvk::CommandBuffer::Create(command_pool);
	command_buffer->Begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);
	command_buffer->CmdCopy(bvh_nodes_staging_buffer, m_bvh_nodes_buffer,
	                        {{0, 0, bvh_nodes_staging_buffer->GetSize()}});
	command_buffer->CmdCopy(bvh_tri_indices_staging_buffer, m_bvh_tri_indices_buffer,
	                        {{0, 0, bvh_tri_indices_staging_buffer->GetSize()}});
	command_buffer->CmdCopy(bvh_tri_matrices_staging_buffer, m_bvh_tri_matrices_buffer,
	                        {{0, 0, bvh_tri_matrices_staging_buffer->GetSize()}});
	command_buffer->End();
	command_buffer->Submit(fence);

	fence->Wait();

	spdlog::info("bvh buffers generated");
}

void AcceleratedScene::create_descriptors(const std::shared_ptr<myvk::Device> &device) {
	{
		VkDescriptorSetLayoutBinding textures_layout_binding = {};
		textures_layout_binding.binding = 0;
		textures_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		textures_layout_binding.descriptorCount = std::max((uint32_t)m_textures.size(), 1u);
		textures_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;

		VkDescriptorSetLayoutBinding triangles_layout_binding = {};
		triangles_layout_binding.binding = 1;
		triangles_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		triangles_layout_binding.descriptorCount = 1;
		triangles_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;

		VkDescriptorSetLayoutBinding tri_materials_layout_binding = {};
		tri_materials_layout_binding.binding = 2;
		tri_materials_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		tri_materials_layout_binding.descriptorCount = 1;
		tri_materials_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;

		VkDescriptorSetLayoutBinding bvh_nodes_layout_binding = {};
		bvh_nodes_layout_binding.binding = 3;
		bvh_nodes_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		bvh_nodes_layout_binding.descriptorCount = 1;
		bvh_nodes_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;

		VkDescriptorSetLayoutBinding bvh_tri_indices_layout_binding = {};
		bvh_tri_indices_layout_binding.binding = 4;
		bvh_tri_indices_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		bvh_tri_indices_layout_binding.descriptorCount = 1;
		bvh_tri_indices_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;

		VkDescriptorSetLayoutBinding bvh_tri_matrices_layout_binding = {};
		bvh_tri_matrices_layout_binding.binding = 5;
		bvh_tri_matrices_layout_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		bvh_tri_matrices_layout_binding.descriptorCount = 1;
		bvh_tri_matrices_layout_binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT | VK_SHADER_STAGE_COMPUTE_BIT;

		m_descriptor_set_layout = myvk::DescriptorSetLayout::Create(
		    device, {textures_layout_binding, triangles_layout_binding, tri_materials_layout_binding,
		             bvh_nodes_layout_binding, bvh_tri_indices_layout_binding, bvh_tri_matrices_layout_binding});
	}
	m_descriptor_pool = myvk::DescriptorPool::Create(
	    device, 1,
	    {{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, std::max((uint32_t)m_textures.size(), 1u)},
	     {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 5}});
	m_descriptor_set = myvk::DescriptorSet::Create(m_descriptor_pool, m_descriptor_set_layout);

	// write textures
	if (!m_textures.empty()) {
		std::vector<VkDescriptorImageInfo> image_infos(m_textures.size());
		for (uint32_t i = 0; i < m_textures.size(); ++i) {
			image_infos[i].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
			image_infos[i].imageView = m_textures[i].m_image_view->GetHandle();
			image_infos[i].sampler = m_sampler->GetHandle();
		}

		VkWriteDescriptorSet write = {};
		write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		write.dstSet = m_descriptor_set->GetHandle();
		write.dstBinding = 0;
		write.dstArrayElement = 0;
		write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		write.descriptorCount = image_infos.size();
		write.pImageInfo = image_infos.data();

		vkUpdateDescriptorSets(device->GetHandle(), 1, &write, 0, nullptr);
	}

	// write storage buffers
	m_descriptor_set->UpdateStorageBuffer(m_triangles_buffer, 1);
	m_descriptor_set->UpdateStorageBuffer(m_tri_materials_buffer, 2);
	m_descriptor_set->UpdateStorageBuffer(m_bvh_nodes_buffer, 3);
	m_descriptor_set->UpdateStorageBuffer(m_bvh_tri_indices_buffer, 4);
	m_descriptor_set->UpdateStorageBuffer(m_bvh_tri_matrices_buffer, 5);
}
