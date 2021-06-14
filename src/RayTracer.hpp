#ifndef RAY_TRACER_HPP
#define RAY_TRACER_HPP

#include <myvk/CommandBuffer.hpp>
#include <myvk/GraphicsPipeline.hpp>

#include "Config.hpp"

#include "AcceleratedScene.hpp"
#include "Camera.hpp"

class RayTracer {
private:
	std::shared_ptr<Camera> m_camera_ptr;
	std::shared_ptr<AcceleratedScene> m_accelerated_scene_ptr;

	uint32_t m_width{kDefaultWidth}, m_height{kDefaultHeight};

	std::shared_ptr<myvk::PipelineLayout> m_pipeline_layout;
	std::shared_ptr<myvk::GraphicsPipeline> m_graphics_pipeline;

	void create_pipeline_layout(const std::shared_ptr<myvk::Device> &device);
	void create_graphics_pipeline(const std::shared_ptr<myvk::RenderPass> &render_pass, uint32_t subpass);

public:
	static std::shared_ptr<RayTracer> Create(const std::shared_ptr<AcceleratedScene> &accelerated_scene,
	                                         const std::shared_ptr<Camera> &camera,
	                                         const std::shared_ptr<myvk::RenderPass> &render_pass, uint32_t subpass);
	void Resize(uint32_t width, uint32_t height);
	void CmdDrawPipeline(const std::shared_ptr<myvk::CommandBuffer> &command_buffer, uint32_t current_frame) const;

	const std::shared_ptr<Camera> &GetCameraPtr() const { return m_camera_ptr; }
	const std::shared_ptr<AcceleratedScene> &GetAcceleratedScenePtr() const { return m_accelerated_scene_ptr; }
};

#endif
