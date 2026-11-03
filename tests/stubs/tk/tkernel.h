#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stddef.h>
#include <stdint.h>
typedef unsigned int UINT;
typedef int INT;
typedef unsigned char UB;
typedef int ID;
typedef struct { void *exinf; uint32_t mtxatr; int ceilpri; } T_CMTX;
#define TA_INHERIT 2u
#define E_OK 0
#define TMO_FEVR (-1)
int tk_cre_mtx(const T_CMTX *c);
int tk_loc_mtx(ID id, int timeout);
int tk_unl_mtx(ID id);
/* Host-only interrupt stubs for the actual IR module integration test. */
#define DI(state) ((state) = 0u)
#define EI(state) ((void)(state))
#endif
