#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stddef.h>
#include <stdint.h>
typedef unsigned int UINT;
typedef int INT;
typedef unsigned char UB;
/* Host-only interrupt stubs for the actual IR module integration test. */
#define DI(state) ((state) = 0u)
#define EI(state) ((void)(state))
#endif
