// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
//
// ==============================================================
/***************************** Include Files *********************************/
#include "xpso_fsm.h"

/************************** Function Implementation *************************/
#ifndef __linux__
int XPso_fsm_CfgInitialize(XPso_fsm *InstancePtr, XPso_fsm_Config *ConfigPtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(ConfigPtr != NULL);

    InstancePtr->Control_BaseAddress = ConfigPtr->Control_BaseAddress;
    InstancePtr->Control_r_BaseAddress = ConfigPtr->Control_r_BaseAddress;
    InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

    return XST_SUCCESS;
}
#endif

void XPso_fsm_Start(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL) & 0x80;
    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL, Data | 0x01);
}

u32 XPso_fsm_IsDone(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL);
    return (Data >> 1) & 0x1;
}

u32 XPso_fsm_IsIdle(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL);
    return (Data >> 2) & 0x1;
}

u32 XPso_fsm_IsReady(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL);
    // check ap_start to see if the pcore is ready for next input
    return !(Data & 0x1);
}

void XPso_fsm_EnableAutoRestart(XPso_fsm *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL, 0x80);
}

void XPso_fsm_DisableAutoRestart(XPso_fsm *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_CTRL, 0);
}

u32 XPso_fsm_Get_return(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_AP_RETURN);
    return Data;
}
void XPso_fsm_Set_u_curr(XPso_fsm *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_U_CURR_DATA, Data);
}

u32 XPso_fsm_Get_u_curr(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_U_CURR_DATA);
    return Data;
}

void XPso_fsm_Set_x_curr(XPso_fsm *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_X_CURR_DATA, Data);
}

u32 XPso_fsm_Get_x_curr(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_X_CURR_DATA);
    return Data;
}

void XPso_fsm_Set_xref(XPso_fsm *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_XREF_DATA, Data);
}

u32 XPso_fsm_Get_xref(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_XREF_DATA);
    return Data;
}

void XPso_fsm_Set_last_best(XPso_fsm *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_LAST_BEST_DATA, Data);
}

u32 XPso_fsm_Get_last_best(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_LAST_BEST_DATA);
    return Data;
}

void XPso_fsm_Set_new_best(XPso_fsm *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_NEW_BEST_DATA, Data);
}

u32 XPso_fsm_Get_new_best(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_NEW_BEST_DATA);
    return Data;
}

void XPso_fsm_Set_bestfitness_top(XPso_fsm *InstancePtr, u32 Data) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_BESTFITNESS_TOP_DATA, Data);
}

u32 XPso_fsm_Get_bestfitness_top(XPso_fsm *InstancePtr) {
    u32 Data;

    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Data = XPso_fsm_ReadReg(InstancePtr->Control_r_BaseAddress, XPSO_FSM_CONTROL_R_ADDR_BESTFITNESS_TOP_DATA);
    return Data;
}

void XPso_fsm_InterruptGlobalEnable(XPso_fsm *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_GIE, 1);
}

void XPso_fsm_InterruptGlobalDisable(XPso_fsm *InstancePtr) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_GIE, 0);
}

void XPso_fsm_InterruptEnable(XPso_fsm *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_IER);
    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_IER, Register | Mask);
}

void XPso_fsm_InterruptDisable(XPso_fsm *InstancePtr, u32 Mask) {
    u32 Register;

    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    Register =  XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_IER);
    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_IER, Register & (~Mask));
}

void XPso_fsm_InterruptClear(XPso_fsm *InstancePtr, u32 Mask) {
    Xil_AssertVoid(InstancePtr != NULL);
    Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    XPso_fsm_WriteReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_ISR, Mask);
}

u32 XPso_fsm_InterruptGetEnabled(XPso_fsm *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_IER);
}

u32 XPso_fsm_InterruptGetStatus(XPso_fsm *InstancePtr) {
    Xil_AssertNonvoid(InstancePtr != NULL);
    Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    return XPso_fsm_ReadReg(InstancePtr->Control_BaseAddress, XPSO_FSM_CONTROL_ADDR_ISR);
}
