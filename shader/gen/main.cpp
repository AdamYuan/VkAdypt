//USAGE: ./gen [INCLUDE PATH] [INPUT FILE] [OUTPUT FILE]
#define STB_INCLUDE_LINE_NONE
#define STB_INCLUDE_IMPLEMENTATION
#include "stb_include.h"

#include <fstream>

int main(int argc, char **argv) {
	--argc;
	++argv;

	if (argc != 3) return EXIT_FAILURE;

	char error[256] = {};
	char *included = stb_include_file(argv[1], nullptr, argv[0], error);

	if (strlen(error)) {
		puts(error);
		putchar('\n');
	}

	std::ofstream out(argv[2]);
	if (!out.is_open()) {
		printf("Error when writing to %s\n", argv[2]);
		return EXIT_FAILURE;
	}
	out << included;
	out.close();

	free(included);

	return EXIT_SUCCESS;
}
