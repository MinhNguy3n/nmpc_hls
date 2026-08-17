// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
//
// ==============================================================
#ifndef __linux__

#include "xstatus.h"
#ifdef SDT
#include "xparameters.h"
#endif
#include "xpso_fsm.h"

extern XPso_fsm_Config XPso_fsm_ConfigTable[];

#ifdef SDT
XPso_fsm_Config *XPso_fsm_LookupConfig(UINTPTR BaseAddress) {
	XPso_fsm_Config *ConfigPtr = NULL;

	int Index;

	for (Index = (u32)0x0; XPso_fsm_ConfigTable[Index].Name != NULL; Index++) {
		if (!BaseAddress || XPso_fsm_ConfigTable[Index].Control_BaseAddress == BaseAddress) {
			ConfigPtr = &XPso_fsm_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XPso_fsm_Initialize(XPso_fsm *InstancePtr, UINTPTR BaseAddress) {
	XPso_fsm_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XPso_fsm_LookupConfig(BaseAddress);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XPso_fsm_CfgInitialize(InstancePtr, ConfigPtr);
}
#else
XPso_fsm_Config *XPso_fsm_LookupConfig(u16 DeviceId) {
	XPso_fsm_Config *ConfigPtr = NULL;

	int Index;

	for (Index = 0; Index < XPAR_XPSO_FSM_NUM_INSTANCES; Index++) {
		if (XPso_fsm_ConfigTable[Index].DeviceId == DeviceId) {
			ConfigPtr = &XPso_fsm_ConfigTable[Index];
			break;
		}
	}

	return ConfigPtr;
}

int XPso_fsm_Initialize(XPso_fsm *InstancePtr, u16 DeviceId) {
	XPso_fsm_Config *ConfigPtr;

	Xil_AssertNonvoid(InstancePtr != NULL);

	ConfigPtr = XPso_fsm_LookupConfig(DeviceId);
	if (ConfigPtr == NULL) {
		InstancePtr->IsReady = 0;
		return (XST_DEVICE_NOT_FOUND);
	}

	return XPso_fsm_CfgInitialize(InstancePtr, ConfigPtr);
}
#endif

#endif
