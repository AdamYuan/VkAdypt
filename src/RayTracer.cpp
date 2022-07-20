#include "RayTracer.hpp"

#include "QuadSpirv.hpp"
#include <myvk/ShaderModule.hpp>

std::shared_ptr<RayTracer> RayTracer::Create(const std::shared_ptr<AcceleratedScene> &accelerated_scene,
                                             const std::shared_ptr<Camera> &camera,
                                             const std::shared_ptr<myvk::RenderPass> &render_pass, uint32_t subpass) {
	std::shared_ptr<RayTracer> ret = std::make_shared<RayTracer>();
	ret->m_accelerated_scene_ptr = accelerated_scene;
	ret->m_camera_ptr = camera;

	ret->create_pipeline_layout(render_pass->GetDevicePtr());
	ret->create_graphics_pipeline(render_pass, subpass);

	return ret;
}

void RayTracer::create_pipeline_layout(const std::shared_ptr<myvk::Device> &device) {
	m_pipeline_layout = myvk::PipelineLayout::Create(
	    device, {m_accelerated_scene_ptr->GetDescriptorSetLayout(), m_camera_ptr->GetDescriptorSetLayout()},
	    {{VK_SHADER_STAGE_FRAGMENT_BIT, 0, 2 * sizeof(uint32_t)}});
}

void RayTracer::create_graphics_pipeline(const std::shared_ptr<myvk::RenderPass> &render_pass, uint32_t subpass) {
	constexpr uint32_t kRayTracerFragSpv[] = {
#include <spirv/ray_tracer.frag.u32>
	};
	std::shared_ptr<myvk::Device> device = render_pass->GetDevicePtr();

	std::shared_ptr<myvk::ShaderModule> vert_shader_module, frag_shader_module;
	vert_shader_module = myvk::ShaderModule::Create(device, kQuadVertSpv, sizeof(kQuadVertSpv));
	frag_shader_module = myvk::ShaderModule::Create(device, kRayTracerFragSpv, sizeof(kRayTracerFragSpv));

	std::vector<VkPipelineShaderStageCreateInfo> shader_stages = {
	    vert_shader_module->GetPipelineShaderStageCreateInfo(VK_SHADER_STAGE_VERTEX_BIT),
	    frag_shader_module->GetPipelineShaderStageCreateInfo(VK_SHADER_STAGE_FRAGMENT_BIT)};

	myvk::GraphicsPipelineState pipeline_state = {};
	pipeline_state.m_vertex_input_state.Enable();
	pipeline_state.m_input_assembly_state.Enable(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	pipeline_state.m_viewport_state.Enable(1, 1);
	pipeline_state.m_rasterization_state.Initialize(VK_POLYGON_MODE_FILL, VK_FRONT_FACE_COUNTER_CLOCKWISE,
	                                                VK_CULL_MODE_FRONT_BIT);
	pipeline_state.m_multisample_state.Enable(VK_SAMPLE_COUNT_1_BIT);
	pipeline_state.m_color_blend_state.Enable(1, VK_FALSE);
	pipeline_state.m_dynamic_state.Enable({VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR});
	m_graphics_pipeline =
	    myvk::GraphicsPipeline::Create(m_pipeline_layout, render_pass, shader_stages, pipeline_state, subpass);
}

void RayTracer::Resize(uint32_t width, uint32_t height) {
	m_width = width;
	m_height = height;
}

void RayTracer::CmdDrawPipeline(const std::shared_ptr<myvk::CommandBuffer> &command_buffer,
                                uint32_t current_frame) const {
	command_buffer->CmdBindPipeline(m_graphics_pipeline);
	command_buffer->CmdBindDescriptorSets(
	    {m_accelerated_scene_ptr->GetDescriptorSet(), m_camera_ptr->GetFrameDescriptorSet(current_frame)},
	    m_graphics_pipeline);

	VkRect2D scissor = {};
	scissor.extent = {m_width, m_height};
	command_buffer->CmdSetScissor({scissor});
	VkViewport viewport = {};
	viewport.width = m_width;
	viewport.height = m_height;
	command_buffer->CmdSetViewport({viewport});

	uint32_t push_constants[] = {m_width, m_height};
	command_buffer->CmdPushConstants(m_pipeline_layout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(push_constants),
	                                 push_constants);
	command_buffer->CmdDraw(3, 1, 0, 0);
}
