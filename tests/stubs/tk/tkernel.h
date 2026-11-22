#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stddef.h>
#include <stdint.h>
typedef unsigned int UINT;
typedef int INT;
typedef unsigned char UB;
typedef int ID;
typedef int ER;
typedef int TMO;
typedef struct { void *exinf; uint32_t mtxatr; int ceilpri; } T_CMTX;
typedef struct { void *exinf; UINT flgatr; UINT iflgptn; } T_CFLG;
#define TA_INHERIT 2u
#define E_OK 0
#define E_TMOUT (-50)
#define TMO_FEVR (-1)
#define TWF_ORW 1u
#define TWF_BITCLR 0x20u
int tk_cre_mtx(const T_CMTX *c);
int tk_loc_mtx(ID id, int timeout);
int tk_unl_mtx(ID id);
ID tk_cre_flg(const T_CFLG *settings);
ER tk_del_flg(ID id);
ER tk_set_flg(ID id, UINT pattern);
ER tk_wai_flg(ID id, UINT requested, UINT mode, UINT *received, int timeout);
/* Host-only interrupt stubs for the actual IR module integration test. */
#define DI(state) ((state) = 0u)
#define EI(state) ((void)(state))
#endif
