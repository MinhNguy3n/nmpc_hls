// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
//
// ==============================================================
#ifndef XPSO_FSM_H
#define XPSO_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#ifndef __linux__
#include "xil_types.h"
#include "xil_assert.h"
#include "xstatus.h"
#include "xil_io.h"
#else
#include <stdint.h>
#include <assert.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#endif
#include "xpso_fsm_hw.h"

/**************************** Type Definitions ******************************/
#ifdef __linux__
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
#else
typedef struct {
#ifdef SDT
    char *Name;
#else
    u16 DeviceId;
#endif
    u32 Control_BaseAddress;
    u32 Control_r_BaseAddress;
} XPso_fsm_Config;
#endif

typedef struct {
    u32 Control_BaseAddress;
    u32 Control_r_BaseAddress;
    u32 IsReady;
} XPso_fsm;

typedef u32 word_type;

/***************** Macros (Inline Functions) Definitions *********************/
#ifndef __linux__
#define XPso_fsm_WriteReg(BaseAddress, RegOffset, Data) \
    Xil_Out32((BaseAddress) + (RegOffset), (u32)(Data))
#define XPso_fsm_ReadReg(BaseAddress, RegOffset) \
    Xil_In32((BaseAddress) + (RegOffset))
#else
#define XPso_fsm_WriteReg(BaseAddress, RegOffset, Data) \
    *(volatile u32*)((BaseAddress) + (RegOffset)) = (u32)(Data)
#define XPso_fsm_ReadReg(BaseAddress, RegOffset) \
    *(volatile u32*)((BaseAddress) + (RegOffset))

#define Xil_AssertVoid(expr)    assert(expr)
#define Xil_AssertNonvoid(expr) assert(expr)

#define XST_SUCCESS             0
#define XST_DEVICE_NOT_FOUND    2
#define XST_OPEN_DEVICE_FAILED  3
#define XIL_COMPONENT_IS_READY  1
#endif

/************************** Function Prototypes *****************************/
#ifndef __linux__
#ifdef SDT
int XPso_fsm_Initialize(XPso_fsm *InstancePtr, UINTPTR BaseAddress);
XPso_fsm_Config* XPso_fsm_LookupConfig(UINTPTR BaseAddress);
#else
int XPso_fsm_Initialize(XPso_fsm *InstancePtr, u16 DeviceId);
XPso_fsm_Config* XPso_fsm_LookupConfig(u16 DeviceId);
#endif
int XPso_fsm_CfgInitialize(XPso_fsm *InstancePtr, XPso_fsm_Config *ConfigPtr);
#else
int XPso_fsm_Initialize(XPso_fsm *InstancePtr, const char* InstanceName);
int XPso_fsm_Release(XPso_fsm *InstancePtr);
#endif

void XPso_fsm_Start(XPso_fsm *InstancePtr);
u32 XPso_fsm_IsDone(XPso_fsm *InstancePtr);
u32 XPso_fsm_IsIdle(XPso_fsm *InstancePtr);
u32 XPso_fsm_IsReady(XPso_fsm *InstancePtr);
void XPso_fsm_EnableAutoRestart(XPso_fsm *InstancePtr);
void XPso_fsm_DisableAutoRestart(XPso_fsm *InstancePtr);
u32 XPso_fsm_Get_return(XPso_fsm *InstancePtr);

void XPso_fsm_Set_u_curr(XPso_fsm *InstancePtr, u32 Data);
u32 XPso_fsm_Get_u_curr(XPso_fsm *InstancePtr);
void XPso_fsm_Set_x_curr(XPso_fsm *InstancePtr, u32 Data);
u32 XPso_fsm_Get_x_curr(XPso_fsm *InstancePtr);
void XPso_fsm_Set_xref(XPso_fsm *InstancePtr, u32 Data);
u32 XPso_fsm_Get_xref(XPso_fsm *InstancePtr);
void XPso_fsm_Set_last_best(XPso_fsm *InstancePtr, u32 Data);
u32 XPso_fsm_Get_last_best(XPso_fsm *InstancePtr);
void XPso_fsm_Set_new_best(XPso_fsm *InstancePtr, u32 Data);
u32 XPso_fsm_Get_new_best(XPso_fsm *InstancePtr);
void XPso_fsm_Set_bestfitness_top(XPso_fsm *InstancePtr, u32 Data);
u32 XPso_fsm_Get_bestfitness_top(XPso_fsm *InstancePtr);

void XPso_fsm_InterruptGlobalEnable(XPso_fsm *InstancePtr);
void XPso_fsm_InterruptGlobalDisable(XPso_fsm *InstancePtr);
void XPso_fsm_InterruptEnable(XPso_fsm *InstancePtr, u32 Mask);
void XPso_fsm_InterruptDisable(XPso_fsm *InstancePtr, u32 Mask);
void XPso_fsm_InterruptClear(XPso_fsm *InstancePtr, u32 Mask);
u32 XPso_fsm_InterruptGetEnabled(XPso_fsm *InstancePtr);
u32 XPso_fsm_InterruptGetStatus(XPso_fsm *InstancePtr);

#ifdef __cplusplus
}
#endif

#endif
