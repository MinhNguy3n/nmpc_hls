// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2024.2 (64-bit)
// Tool Version Limit: 2024.11
// Copyright 1986-2022 Xilinx, Inc. All Rights Reserved.
// Copyright 2022-2024 Advanced Micro Devices, Inc. All Rights Reserved.
//
// ==============================================================
// control
// 0x00 : Control signals
//        bit 0  - ap_start (Read/Write/COH)
//        bit 1  - ap_done (Read/COR)
//        bit 2  - ap_idle (Read)
//        bit 3  - ap_ready (Read/COR)
//        bit 7  - auto_restart (Read/Write)
//        bit 9  - interrupt (Read)
//        others - reserved
// 0x04 : Global Interrupt Enable Register
//        bit 0  - Global Interrupt Enable (Read/Write)
//        others - reserved
// 0x08 : IP Interrupt Enable Register (Read/Write)
//        bit 0 - enable ap_done interrupt (Read/Write)
//        bit 1 - enable ap_ready interrupt (Read/Write)
//        others - reserved
// 0x0c : IP Interrupt Status Register (Read/TOW)
//        bit 0 - ap_done (Read/TOW)
//        bit 1 - ap_ready (Read/TOW)
//        others - reserved
// 0x10 : Data signal of ap_return
//        bit 31~0 - ap_return[31:0] (Read)
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XPSO_FSM_CONTROL_ADDR_AP_CTRL   0x00
#define XPSO_FSM_CONTROL_ADDR_GIE       0x04
#define XPSO_FSM_CONTROL_ADDR_IER       0x08
#define XPSO_FSM_CONTROL_ADDR_ISR       0x0c
#define XPSO_FSM_CONTROL_ADDR_AP_RETURN 0x10
#define XPSO_FSM_CONTROL_BITS_AP_RETURN 32

// control_r
// 0x00 : reserved
// 0x04 : reserved
// 0x08 : reserved
// 0x0c : reserved
// 0x10 : Data signal of u_curr
//        bit 31~0 - u_curr[31:0] (Read/Write)
// 0x14 : reserved
// 0x18 : Data signal of x_curr
//        bit 31~0 - x_curr[31:0] (Read/Write)
// 0x1c : reserved
// 0x20 : Data signal of xref
//        bit 31~0 - xref[31:0] (Read/Write)
// 0x24 : reserved
// 0x28 : Data signal of last_best
//        bit 31~0 - last_best[31:0] (Read/Write)
// 0x2c : reserved
// 0x30 : Data signal of new_best
//        bit 31~0 - new_best[31:0] (Read/Write)
// 0x34 : reserved
// 0x38 : Data signal of bestfitness_top
//        bit 31~0 - bestfitness_top[31:0] (Read/Write)
// 0x3c : reserved
// (SC = Self Clear, COR = Clear on Read, TOW = Toggle on Write, COH = Clear on Handshake)

#define XPSO_FSM_CONTROL_R_ADDR_U_CURR_DATA          0x10
#define XPSO_FSM_CONTROL_R_BITS_U_CURR_DATA          32
#define XPSO_FSM_CONTROL_R_ADDR_X_CURR_DATA          0x18
#define XPSO_FSM_CONTROL_R_BITS_X_CURR_DATA          32
#define XPSO_FSM_CONTROL_R_ADDR_XREF_DATA            0x20
#define XPSO_FSM_CONTROL_R_BITS_XREF_DATA            32
#define XPSO_FSM_CONTROL_R_ADDR_LAST_BEST_DATA       0x28
#define XPSO_FSM_CONTROL_R_BITS_LAST_BEST_DATA       32
#define XPSO_FSM_CONTROL_R_ADDR_NEW_BEST_DATA        0x30
#define XPSO_FSM_CONTROL_R_BITS_NEW_BEST_DATA        32
#define XPSO_FSM_CONTROL_R_ADDR_BESTFITNESS_TOP_DATA 0x38
#define XPSO_FSM_CONTROL_R_BITS_BESTFITNESS_TOP_DATA 32
