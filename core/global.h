#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include <stdio.h>

#define NOTICE(str) printf("\033[0;32m%s:%d: %s (NOTICE)\n\033[0m", __FILE__, __LINE__, str);
#define WARNING(str) printf("\033[0;35m%s:%d: %s (WARNING)\n\033[0m", __FILE__, __LINE__, str);
#define ERROR(str) printf("\033[0;31m%s:%d: %s (ERROR)\n\033[0m", __FILE__, __LINE__, str);

#endif