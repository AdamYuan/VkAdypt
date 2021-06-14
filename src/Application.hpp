#ifndef APPLICATION_HPP
#define APPLICATION_HPP

#include <GLFW/glfw3.h>

#include <memory>
#include <vector>

#include <spdlog/sinks/ringbuffer_sink.h>

#include <myvk/FrameManager.hpp>

#include "Camera.hpp"
#include "ImGuiRenderer.hpp"
#include "RayTracer.hpp"

class Application {
private:
	GLFWwindow *m_window{nullptr};

	// base
	std::shared_ptr<myvk::Instance> m_instance;
	std::shared_ptr<myvk::Surface> m_surface;
	std::shared_ptr<myvk::Device> m_device;
	std::shared_ptr<myvk::Queue> m_main_queue, m_loader_queue, m_path_tracer_queue;
	std::shared_ptr<myvk::PresentQueue> m_present_queue;
	std::shared_ptr<myvk::CommandPool> m_main_command_pool, m_path_tracer_command_pool;

	// frame objects
	myvk::FrameManager m_frame_manager;
	std::vector<std::shared_ptr<myvk::Framebuffer>> m_framebuffers;
	std::vector<std::shared_ptr<myvk::CommandBuffer>> m_frame_command_buffers;

	// render pass
	std::shared_ptr<myvk::RenderPass> m_render_pass;

	ImGuiRenderer m_imgui_renderer;

	// global resources
	std::shared_ptr<Camera> m_camera;
	std::shared_ptr<AcceleratedScene> m_accelerated_scene;
	std::shared_ptr<RayTracer> m_ray_tracer;

	// std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> m_log_sink;

	// ui flags
	bool m_ui_display_flag{true};

	void create_window();
	void initialize_vulkan();
	void create_render_pass();
	void create_framebuffers();
	void resize();
	void draw_frame();

	static void glfw_key_callback(GLFWwindow *window, int key, int scancode, int action, int mods);
	static void glfw_framebuffer_resize_callback(GLFWwindow *window, int width, int height);

public:
	Application();
	~Application();
	void Load(const char *filename);
	void Run();
};

#endif
