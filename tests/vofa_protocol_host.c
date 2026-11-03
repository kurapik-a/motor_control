/* LCC的旧stdlib.h缺少C99声明；协议实现仍直接使用SDK源码。 */
#include <stdlib.h>
float strtof(const char *text, char **end);
#include "vofa_debug.c"
