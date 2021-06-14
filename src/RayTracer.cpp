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

	VkPipelineShaderStageCreateInfo shader_stages[] = {
	    vert_shader_module->GetPipelineShaderStageCreateInfo(VK_SHADER_STAGE_VERTEX_BIT),
	    frag_shader_module->GetPipelineShaderStageCreateInfo(VK_SHADER_STAGE_FRAGMENT_BIT)};

	VkPipelineVertexInputStateCreateInfo vertex_input = {};
	vertex_input.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

	VkPipelineInputAssemblyStateCreateInfo input_assembly = {};
	input_assembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	input_assembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	input_assembly.primitiveRestartEnable = VK_FALSE;

	VkPipelineViewportStateCreateInfo viewport_state = {};
	viewport_state.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewport_state.viewportCount = 1;
	viewport_state.scissorCount = 1;

	VkPipelineRasterizationStateCreateInfo rasterizer = {};
	rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterizer.depthClampEnable = VK_FALSE;
	rasterizer.rasterizerDiscardEnable = VK_FALSE;
	rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
	rasterizer.lineWidth = 1.0f;
	rasterizer.cullMode = VK_CULL_MODE_FRONT_BIT;
	rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rasterizer.depthBiasEnable = VK_FALSE;

	VkPipelineMultisampleStateCreateInfo multisampling = {};
	multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	multisampling.sampleShadingEnable = VK_FALSE;
	multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
	multisampling.minSampleShading = 0.0f;
	multisampling.pSampleMask = nullptr;
	multisampling.alphaToCoverageEnable = VK_FALSE;
	multisampling.alphaToOneEnable = VK_FALSE;

	VkPipelineColorBlendAttachmentState color_blend_attachment = {};
	color_blend_attachment.blendEnable = VK_FALSE;
	color_blend_attachment.colorWriteMask =
	    VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT;

	VkPipelineColorBlendStateCreateInfo color_blend = {};
	color_blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	color_blend.attachmentCount = 1;
	color_blend.pAttachments = &color_blend_attachment;

	VkDynamicState dynamic_states[2] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
	VkPipelineDynamicStateCreateInfo dynamic_state = {};
	dynamic_state.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynamic_state.dynamicStateCount = 2;
	dynamic_state.pDynamicStates = dynamic_states;

	VkGraphicsPipelineCreateInfo pipeline_info = {};
	pipeline_info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	pipeline_info.stageCount = 2;
	pipeline_info.pStages = shader_stages;
	pipeline_info.pVertexInputState = &vertex_input;
	pipeline_info.pInputAssemblyState = &input_assembly;
	pipeline_info.pViewportState = &viewport_state;
	pipeline_info.pRasterizationState = &rasterizer;
	pipeline_info.pMultisampleState = &multisampling;
	pipeline_info.pDepthStencilState = nullptr;
	pipeline_info.pColorBlendState = &color_blend;
	pipeline_info.pDynamicState = &dynamic_state;
	pipeline_info.subpass = subpass;

	m_graphics_pipeline = myvk::GraphicsPipeline::Create(m_pipeline_layout, render_pass, pipeline_info);
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
