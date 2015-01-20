#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include <stdio.h>

// These macros are quite unsecure and allow for buffer overflows (sprintf)
// but we assume this won't happen for now...

#define NOTICE(...) { int _line = __LINE__;\
	char _not[512];\
	sprintf(_not, __VA_ARGS__);\
	printf("\033[0;32m%s:%d: %s (NOTICE)\n\033[0m", __FILE__, _line, _not);\
}

#define WARNING(...) { int _line = __LINE__;\
	char _not[512];\
	sprintf(_not, __VA_ARGS__);\
	printf("\033[0;35m%s:%d: %s (WARNING)\n\033[0m", __FILE__, _line, _not);\
}

#define ERROR(...) { int _line = __LINE__;\
	char _not[512];\
	sprintf(_not, __VA_ARGS__);\
	printf("\033[0;31m%s:%d: %s (ERROR)\n\033[0m", __FILE__, _line, _not);\
}

#endif