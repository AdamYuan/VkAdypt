#include "Application.hpp"
#include <spdlog/spdlog.h>

constexpr const char *kHelpStr = "AdamYuan's Path Tracer (Driven by Vulkan)\n"
                                 "\t-obj [WAVEFRONT OBJ FILENAME]\n";

int main(int argc, char **argv) {
	spdlog::set_pattern("[%H:%M:%S.%e] [%^%l%$] [thread %t] %v");

	--argc;
	++argv;
	char **filename = nullptr;
	for (int i = 0; i < argc; ++i) {
		if (i + 1 < argc && strcmp(argv[i], "-obj") == 0)
			filename = argv + i + 1, ++i;
		else {
			printf(kHelpStr);
			return EXIT_FAILURE;
		}
	}

	if (filename == nullptr) {
		printf(kHelpStr);
		return EXIT_FAILURE;
	}

	{
		Application app{};
		app.Load(*filename);
		app.Run();
	}

	return EXIT_SUCCESS;
}
