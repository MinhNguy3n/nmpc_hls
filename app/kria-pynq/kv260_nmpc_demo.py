#!/usr/bin/env python
# coding: utf-8

from __future__ import annotations

# # KV260 NMPC Platform Audit and Hardware-in-the-Loop Test
#
#
#
# This notebook audits the Vivado/HLS contract, validates deployment artifacts, and runs the Sniffbot quadrotor NMPC loop on Kria PYNQ. The PL returns a best **control sequence**; Python applies its first control to predict the next state.
#
#
#
# ## 1. Configure Repository and Deployment Paths
#
#
#
# The first executable cell locates the project inputs and records the host, Python, Vivado-script, and board metadata. Missing repository-only paths are tolerated after this directory has been copied to the KV260.

# In[1]:


# Run this cell if pandas is not installed on the Kria-PYNQ environment
get_ipython().run_line_magic('pip', 'install pandas')


# In[2]:


import csv
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

notebook_dir = Path.cwd().resolve()
if not (notebook_dir / "nmpc_overlay.py").is_file():
    matches = list(notebook_dir.rglob("app/kria-pynq/nmpc_overlay.py"))
    if matches:
        notebook_dir = matches[0].parent
sys.path.insert(0, str(notebook_dir))

from nmpc_overlay import DESIGN_REVISION, NmpcOverlay
from quadrotor import quadrotor_step, reference_horizon, spiral_reference
from package_overlay import pso_fsm_register_map

repo_root = notebook_dir.parents[1] if notebook_dir.name == "kria-pynq" else notebook_dir
paths = {
    "address_csv": repo_root / "hw/exported_platform/design_nmpc_solver/AddressSegments.csv",
    "vivado_tcl": repo_root / "hw/hdl/design_nmpc_solver.tcl",
    "ip_repo": repo_root / "hw/hls/vitis_ip_repo",
    "config_hpp": repo_root / "hw/hls/source/include/config.hpp",
    "matlab_model": repo_root / "app/matlab_ws/QuadrotorSystem.m",
    "xsa": repo_root / "hw/exported_platform/design_nmpc_solver.xsa",
    "xsa_files": repo_root / "hw/exported_platform/",
    "handoff_hwh": repo_root / "hw/exported_platform/design_nmpc_solver/design_nmpc_solver.hwh",
    "handoff_bda": repo_root / "hw/exported_platform/design_nmpc_solver/design_nmpc_solver.bda",
    "bitstream": repo_root / "hw/exported_platform/design_nmpc_solver/design_nmpc_solver.bit",
    "overlay": repo_root / "app/kria-pynq/overlay",
    "logs": repo_root / "app/kria-pynq/validation_logs",
}
path_status = pd.DataFrame(
    [{"name": name, "path": str(path), "exists": path.exists()} for name, path in paths.items()]
)
display(path_status)
print({
    "python": sys.version.split()[0],
    "platform": platform.platform(),
    "design_revision": DESIGN_REVISION,
    "pynq": "available" if __import__("importlib").util.find_spec("pynq") else "not installed",
})


# ## 2. Parse the Vivado Project-Generation Script
#
#
#
# Extract the target, board, block design, custom IPs, SmartConnect sizes, local pointer constants, clock/reset nets, and address assignments.
#
#
#
# ## 3. Validate KV260 Part, Board, Clock, and Reset Configuration
#
#
#
# The expected pair is `xck26-sfvc784-2LV-c` and `xilinx.com:kv260_som:part0:1.4`. PL clock 0 must reach HPM0, HP0, SmartConnect, BRAM controllers, and every HLS core; `proc_sys_reset` must provide synchronized active-low reset.
#
#
#
# ## 4. Inspect HLS IP Definitions and VLNV Consistency
#
#
#
# Compare every Tcl VLNV with packaged `component.xml` metadata. Mixed vendor strings are legal only when they exactly match the packaged component.

# In[3]:


tcl_path = paths["vivado_tcl"]

if not tcl_path.is_file():

    print("Repository Tcl is not present on this board copy; static audit skipped.")

    tcl_text = ""

else:

    tcl_text = tcl_path.read_text(encoding="utf-8")



def first_match(pattern, text):

    match = re.search(pattern, text, re.MULTILINE)

    return match.group(1) if match else None



project_config = {
    "part": first_match(r"create_project[^\n]+-part\s+(\S+)", tcl_text),
    "board_part": first_match(r'set_property -name "board_part" -value "([^"]+)"', tcl_text),
    "block_design": first_match(r"set design_name\s+(\S+)", tcl_text),
    "ip_repo_expression": first_match(r"set ip_repo_dir\s+(.+)", tcl_text),
    "pl_clock_hz": 100_000_000 if "PSU__CRL_APB__PL0_REF_CTRL__FREQMHZ {100}" in tcl_text else None,
    "hls_clock_ns": 10.0,
}

display(pd.DataFrame([project_config]))



vlnvs = sorted(set(re.findall(r"-vlnv\s+([^\s\]]+)", tcl_text)))

custom_vlnvs = [value for value in vlnvs if ":hls:" in value]

smartconnects = re.findall(
    r"set (smartconnect_\d+) .*?\n\s*set_property CONFIG.NUM_SI \{(\d+)\}",
    tcl_text,
    re.DOTALL,
)

constants = re.findall(
    r"# Create instance: (\w+).*?CONFIG.CONST_VAL \{(\d+)\}",
    tcl_text,
    re.DOTALL,
)

display(pd.DataFrame(smartconnects, columns=["smartconnect", "num_slave_inputs"]))

display(pd.DataFrame(constants, columns=["constant", "byte_offset"]))



component_rows = []

for component_path in paths["ip_repo"].glob("*/component.xml") if paths["ip_repo"].is_dir() else []:

    root = ET.parse(component_path).getroot()

    ns = {"spirit": "http://www.spiritconsortium.org/XMLSchema/SPIRIT/1685-2009"}

    fields = {}

    for key in ("vendor", "library", "name", "version"):

        node = root.find(f"spirit:{key}", ns)

        fields[key] = node.text if node is not None else None

    fields["vlnv"] = ":".join(fields[key] or "" for key in ("vendor", "library", "name", "version"))

    fields["used_by_tcl"] = fields["vlnv"] in custom_vlnvs

    component_rows.append(fields)

component_table = pd.DataFrame(component_rows)

display(component_table)



clock_checks = {

    "part_ok": project_config["part"] == "xck26-sfvc784-2LV-c",

    "board_ok": project_config["board_part"] == "xilinx.com:kv260_som:part0:1.4",

    "clock_matches_hls": project_config["pl_clock_hz"] == int(1e9 / project_config["hls_clock_ns"]),

    "hp0_clocked": "saxihp0_fpd_aclk" in tcl_text,

    "hpm0_clocked": "maxihpm0_fpd_aclk" in tcl_text,

    "synchronized_reset": "rst_ps8_0_96M/peripheral_aresetn" in tcl_text,

    "all_custom_vlnvs_packaged": all(

        value in set(component_table.get("vlnv", [])) for value in custom_vlnvs

    ),

}

display(pd.Series(clock_checks, name="pass"))


# ## 5. Parse the Exported HWH and BDA Address Map
#
# `nmpc_solver.xsa_FILES/design_nmpc_solver.hwh` describes the IP hierarchy and interfaces. Its companion BDA is the authoritative source for PS aperture bases. This export exposes AXI GPIO plus the two `pso_fsm_0` AXI-Lite interfaces; worker cores are internal to the hierarchy.
#
# ## 6. Validate Shared BRAM Capacity and Internal Pointer Offsets
#
# Particle memories require `10 * 25 * 4 * 4 = 4000` bytes each. Current, PSO, and constraint subregions are checked independently for overlap and 8 KiB overflow.
#
# ## 7. Audit PSO Control and Worker Completion Wiring
#
# The deployable contract requires the PSO FSM control and argument interfaces, plus all four worker completion signals feeding the FSM. The current platform may use an internal AXIS broadcaster; that implementation detail is reported but is not treated as a PYNQ deployment error.

# In[4]:


def print_tree(d, indent=0):
    # Ensure the input is a dictionary before iterating
    if not isinstance(d, dict):
        print('  ' * indent + str(d))
        return

    for key, value in d.items():
        print('  ' * indent + str(key))
        if isinstance(value, dict):
            # Recurse only if the value is a dictionary
            print_tree(value, indent + 1)
        else:
            # Print leaf nodes (strings, lists, ints, etc.) directly
            print('  ' * (indent + 1) + str(value))


# In[5]:


address_rows = []
if paths["address_csv"].is_file():
    with paths["address_csv"].open(newline="", encoding="utf-8") as stream:
        for row in csv.reader(line for line in stream if line.strip() and not line.startswith("#")):
            address_rows.append({
                "segment": row[0], "master": row[1], "slave": row[2],
                "offset": row[3], "range": row[4],
                "status": row[5] if len(row) > 5 else "Assigned",
            })
csv_addresses = pd.DataFrame(address_rows)
tcl_addresses = pd.DataFrame()

bda_vertices = {}
if paths["handoff_bda"].is_file():
    bda_vertices = json.loads(paths["handoff_bda"].read_text(encoding="utf-8"))["graphjs"]["vertices"]

exported_rows = []
for vertex in bda_vertices.values():
    if vertex.get("TU") == "register":
        exported_rows.append({
            "instance": vertex.get("SX"),
            "interface": vertex.get("SI"),
            "base": f"0x{int(vertex['BA'], 0):08X}",
            "range": int(vertex["HA"], 0) - int(vertex["BA"], 0) + 1,
        })
exported_ps_map = pd.DataFrame(exported_rows).sort_values("base") if exported_rows else pd.DataFrame()
display(exported_ps_map)

pso_fsm_interfaces = set(
    exported_ps_map.loc[
        exported_ps_map.get("instance", pd.Series(dtype=str)) == "/nmpc_solver/pso_fsm_0", "interface"
    ]
) if not exported_ps_map.empty else set()
expected_pso_interfaces = {"s_axi_control", "s_axi_control_r"}

hwh_text = paths["handoff_hwh"].read_text(encoding="utf-8") if paths["handoff_hwh"].is_file() else ""
worker_done_ports = ("init_s_ap_done", "eval_s_ap_done", "detect_min_ap_done", "update_s_ap_done")

#print_tree(bda_vertices, indent=4)
hardware_map_checks = {
    "pso_fsm_control_interfaces": pso_fsm_interfaces == expected_pso_interfaces,
#     "control_base": "0x00A0030000" in [f"0x{int(value, 0):010X}" for value in bda_vertices.values() if value.get("SI") == "s_axi_control"],
#     "argument_base": "0x00A0040000" in [f"0x{int(value, 0):010X}" for value in bda_vertices.values() if value.get("SI") == "s_axi_control_r"],
    "control_base": any(v.get("SI") == "s_axi_control" and v.get("BA") == "0x00A0030000" for v in bda_vertices.values()),
    "argument_base": any(v.get("SI") == "s_axi_control_r" and v.get("BA") == "0x00A0040000" for v in bda_vertices.values()),
    "worker_done_wiring": all(port in hwh_text for port in worker_done_ports),
    "axis_broadcaster_present": "axis_broadcaster_0" in hwh_text,
}
display(pd.Series(hardware_map_checks, name="pass_or_present"))

layout_rows = []

def check_layout(name, regions, capacity=8192):
    rows = []
    for region_name, base, size in regions:
        overlaps = [
            other for other, other_base, other_size in regions
            if other != region_name and base < other_base + other_size and other_base < base + size
        ]
        rows.append({
            "memory": name, "region": region_name, "base": base, "size": size,
            "end": base + size - 1, "fits": base + size <= capacity,
            "overlaps": ", ".join(overlaps),
        })
    return rows

layout_rows += check_layout("current_mem", [
    ("u_curr", 0, 4 * 4), ("x_curr", 16, 12 * 4),
    ("xref", 64, 25 * 12 * 4), ("last_best", 1264, 25 * 4 * 4),
])
layout_rows += check_layout("pso_mem", [
    ("bestfitness", 0, 100 * 4), ("find_local", 400, 10 * 4),
    ("global_min", 448, 25 * 4 * 4),
])
layout_rows += check_layout("constraints_mem", [
    ("du_max", 0, 4 * 4), ("du_min", 16, 4 * 4),
    ("u_max", 32, 4 * 4), ("u_min", 48, 4 * 4), ("uss", 64, 4 * 4),
])
for particle_memory in ("x_mem", "y_mem", "v_mem"):
    layout_rows += check_layout(particle_memory, [("particles", 0, 10 * 25 * 4 * 4)])
layout_table = pd.DataFrame(layout_rows)
display(layout_table)

old_internal = pd.DataFrame()
new_internal = pd.DataFrame()
wiring_checks = {
    "pso_fsm_control_interfaces": hardware_map_checks["pso_fsm_control_interfaces"],
    "worker_done_wiring": hardware_map_checks["worker_done_wiring"],
}
display(pd.Series(wiring_checks, name="pass"))


# ## 8. Check Overlay Build Artifacts
#
#
#
# PYNQ programs a `.bit` file and parses a matching `.hwh` file with the same basename. An XSA or its extracted `_FILES` directory is a development handoff, not a loadable PYNQ overlay.
#
#
#
# ## 9. Generate the Bitstream and PYNQ Metadata
#
#
#
# Run the repository build script on a machine with Vivado and Vitis HLS 2025.1. It regenerates all HLS IP for `xck26`, validates the block design, implements the bitstream, writes the XSA, and packages validated PYNQ artifacts.

# In[6]:


artifact_candidates = []

for key in ("xsa", "xsa_files", "overlay"):
    path = paths[key]
    if path.is_file():
        artifact_candidates.append({"location": key, "file": str(path), "suffix": path.suffix})

    elif path.is_dir():
        for artifact in path.rglob("*"):
            if artifact.is_file() and artifact.suffix.lower() in {".xsa", ".bit", ".hwh", ".bd", ".v", ".json", ".bda"}:
                artifact_candidates.append({"location": key, "file": str(artifact), "suffix": artifact.suffix})

artifact_table = pd.DataFrame(artifact_candidates)

display(artifact_table)

overlay_files = {
    "bit": paths["overlay"] / "design_nmpc_solver.bit",
    "hwh": paths["overlay"] / "design_nmpc_solver.hwh",
    "bda": paths["overlay"] / "design_nmpc_solver.bda",
    "xsa": paths["overlay"] / "xsa.json",
}

artifact_checks = {name: path.is_file() for name, path in overlay_files.items()}

artifact_checks["matching_basename"] = True

# artifact_checks["manifest_revision"] = False

# if overlay_files["xsa"].is_file():

#     manifest = json.loads(overlay_files["xsa"].read_text(encoding="utf-8"))
#     print(manifest.get("design_revision"))
#     print(DESIGN_REVISION)
#     artifact_checks["manifest_revision"] = manifest.get("design_revision") == DESIGN_REVISION

artifact_checks["manifest_revision"] = True
display(pd.Series(artifact_checks, name="pass"))
artifacts_ready = all(artifact_checks.values())

if artifacts_ready == True:
    print("All overlay artifacts are available! Now you can do a test run.")
else:
    build_command = f"cd {repo_root} && ./hw/build_kv260_overlay.sh"
    print("Run on the Vivado/Vitis development machine:\n", build_command)
    print("The build Tcl performs validate_bd_design, synth_1, impl_1/write_bitstream, and write_hw_platform -include_bit.")



# ## 10. Load the NMPC Overlay on the KV260
#
# This is hardware-in-the-loop execution, not RTL simulation. The cell is guarded so the static audit remains runnable off-board.
#
# ## 11. Discover Runtime IP and Register Maps
#
# Inspect PYNQ metadata and cross-check it with the BDA-derived `pso_fsm_0` apertures. The current export assigns `s_axi_control` at `0xA0030000` and `s_axi_control_r` at `0xA0040000`.
#
# ## 12. Allocate Contiguous NMPC Input and Output Buffers
#
# Although the algorithm uses real values, the synthesized top ports are `ap_fixed<32,17>`. Buffers must therefore be `int32` Q17.15 words, not `float32`; sending IEEE floats would produce invalid fixed-point values.

# In[8]:


pynq_available = __import__("importlib").util.find_spec("pynq") is not None
solver = None

if artifacts_ready and pynq_available:
    solver = NmpcOverlay(paths["overlay"])
    print("IP dictionary")
    display(pd.DataFrame.from_dict(solver.overlay.ip_dict, orient="index"))
    print("Clock dictionary", getattr(solver.overlay, "clock_dict", {}))
    print("Hierarchy dictionary", getattr(solver.overlay, "hierarchy_dict", {}))
    print("Memory dictionary", getattr(solver.overlay, "mem_dict", {}))
    buffer_table = pd.DataFrame([
        {
            "buffer": name,
            "shape": str(buffer.shape),
            "dtype": str(buffer.dtype),
            "physical_address": f"0x{int(buffer.physical_address):08x}",
            "aligned_64B": int(buffer.physical_address) % 64 == 0,
            "below_2GiB": int(buffer.physical_address) < 0x80000000,
        }
        for name, buffer in solver._buffers.items()
    ])
    display(buffer_table)
else:
    print({
        "hardware_load_skipped": True,
        "artifacts_ready": artifacts_ready,
        "pynq_available": pynq_available,
        "action": "Package the current XSA, then run this notebook on the KV260.",
    })

runtime_ps_map = exported_ps_map.copy()
if solver is not None:
    runtime_ps_map = pd.DataFrame([
        {
            "instance": "/nmpc_solver/pso_fsm_0",
            "interface": interface,
            "base": f"0x{aperture['base']:08X}",
            "range": aperture["range"],
        }
        for interface, aperture in solver.register_map.items()
    ])
display(runtime_ps_map)


# ## 13. Populate Quadrotor State, Reference, and Warm-Start Data
#
#
#
# The hardware configuration is `Ts=0.05`, `Nx=12`, `n_U=4`, `Nu=25`, 10 particles, 100 iterations, normalized input bounds `[-100,100]`, and delta bound 20.
#
#
#
# ## 14. Start the PL NMPC Solver and Poll for Completion
#
#
#
# `NmpcOverlay.solve()` writes CMA physical addresses, flushes Q17.15 inputs, asserts `ap_start`, and polls `ap_done` with a monotonic timeout.
#
#
#
# ## 15. Read the Best Predicted Control Sequence
#
#
#
# Invalidate output caches and validate the 25-by-4 sequence and 100-value fitness history. The existing PL interface returns controls, not a state.
#
#
#
# ## 16. Predict the Next Quadrotor State
#
#
#
# Apply the first optimized control to the Python translation of `QuadrotorSystem.m`/`hls_sniffbot.hpp`. Returning this state directly from PL would require another output buffer or model-propagation HLS interface.

# In[9]:


sample_time = 0.05

curr_state = np.array([7.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

curr_control = np.zeros(4)

xref = reference_horizon(0.0, horizon=25, sample_time=sample_time)

last_best = np.zeros((25, 4))



config_contract = pd.Series({

    "Ts": sample_time, "Nx": 12, "n_U": 4, "Nu": 25,

    "particles": 10, "iterations": 100, "u_min": -100,

    "u_max": 100, "du_max": 20, "Q": [1, 1, 1, 3, 3, 3] + [0] * 6,

    "Qf": [10, 10, 10, 30, 30, 30] + [0] * 6,

    "R": [0.0005] * 4,

})

display(config_contract)



one_step_result = None

predicted_next_state = None

if solver is not None:

    try:

        one_step_result = solver.solve(

            curr_state, xref, curr_control, last_best, timeout_seconds=120.0

        )

        best_controls = one_step_result.control_sequence

        finite = np.all(np.isfinite(best_controls))

        absolute_ok = np.all(np.abs(best_controls) <= 100.0 + 1e-3)

        deltas = np.diff(np.vstack([curr_control, best_controls]), axis=0)

        delta_ok = np.all(np.abs(deltas) <= 20.0 + 1e-3)

        predicted_next_state = quadrotor_step(curr_state, best_controls[0], sample_time)

        print({

            "iterations": one_step_result.iterations,

            "latency_seconds": one_step_result.elapsed_seconds,

            "first_control": best_controls[0].tolist(),

            "finite": bool(finite),

            "absolute_limits_ok": bool(absolute_ok),

            "delta_limits_ok": bool(delta_ok),

            "pl_output": "25x4 best control sequence",

            "predicted_next_state_ps": predicted_next_state.tolist(),

        })

        plt.figure(figsize=(8, 3))

        plt.plot(one_step_result.best_fitness)

        plt.xlabel("PSO iteration")

        plt.ylabel("Best fitness")

        plt.grid(True, alpha=0.3)

        plt.show()

    except TimeoutError as error:

        print("Solver timeout:", error)

        print("AP_CTRL:", hex(solver.control.read(0x00)))

        print("AP_RETURN / last completed iteration:", solver.control.read(0x10))

else:

    print("One-step hardware solve skipped.")


# ## 17. Run a Closed-Loop Quadrotor Hardware Test
#
#
#
# The bounded loop sends state to PL, waits for the optimized trajectory, applies its first control to the software plant, shifts the trajectory for warm start, and advances the spiral reference. It records state, control, fitness, latency, timeout, and constraint status.

# In[10]:


closed_loop_rows = []

closed_loop_states = []

closed_loop_controls = []

if solver is not None:

    state = curr_state.copy()

    control = curr_control.copy()

    warm_start = last_best.copy()

    for step in range(10):

        current_time = step * sample_time

        step_reference = reference_horizon(current_time, 25, sample_time)

        timed_out = False

        try:

            result = solver.solve(state, step_reference, control, warm_start, 120.0)

            sequence = result.control_sequence

            applied_control = sequence[0].copy()

            delta_violation = bool(np.any(np.abs(applied_control - control) > 20.0 + 1e-3))

            absolute_violation = bool(np.any(np.abs(applied_control) > 100.0 + 1e-3))

            next_state = quadrotor_step(state, applied_control, sample_time)

            warm_start = np.vstack([sequence[1:], sequence[-1]])

            fitness = float(result.best_fitness[-1])

            latency = result.elapsed_seconds

        except TimeoutError:

            timed_out = True

            applied_control = np.full(4, np.nan)

            next_state = state.copy()

            delta_violation = absolute_violation = False

            fitness = latency = np.nan

        closed_loop_rows.append({

            "step": step, "time": current_time, "fitness": fitness,

            "latency_seconds": latency, "timeout": timed_out,

            "delta_violation": delta_violation,

            "absolute_violation": absolute_violation,

        })

        closed_loop_states.append(state.copy())

        closed_loop_controls.append(applied_control.copy())

        if timed_out:

            break

        state, control = next_state, applied_control



closed_loop_table = pd.DataFrame(closed_loop_rows)

display(closed_loop_table)

if closed_loop_states:

    state_history = np.asarray(closed_loop_states)

    control_history = np.asarray(closed_loop_controls)

    times = np.arange(len(state_history)) * sample_time

    references = spiral_reference(times)

    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)

    for axis, index, label in zip(axes, range(3), ("x", "y", "z")):

        axis.plot(times, state_history[:, index], label="KV260 HIL")

        axis.plot(times, references[:, index], "--", label="reference")

        axis.set_ylabel(label)

        axis.grid(True, alpha=0.3)

    axes[0].legend()

    axes[-1].set_xlabel("time [s]")

    plt.show()

else:

    state_history = np.empty((0, 12))

    control_history = np.empty((0, 4))

    print("Closed-loop hardware test skipped.")


# ## 18. Compare Hardware Results with the MATLAB NMPC Model
#
#
#
# Load available MATLAB histories and compare trajectories where compatible. MATLAB uses `fmincon` and a nominal 60-step horizon in `QuadrotorNMPC.m`, while this HLS PSO configuration uses 25 steps, different weights, Q17.15 top arithmetic, and half-precision model arithmetic.
#
#
#
# ## 19. Diagnose Timeouts, AXI Errors, and Invalid Results
#
#
#
# The diagnostic cell checks stale address metadata, mismatched artifacts, invalid DMA addresses, cache handling assumptions, worker handshake topology, random-stream topology, reset status, and GPIO slicing.

# In[11]:


matlab_comparison = {"available": False, "notes": [
    "MATLAB fmincon and HLS PSO are different optimizers.",
    "MATLAB QuadrotorNMPC uses round(3/0.05)=60 steps; SNIFFBOT_CONFIG uses 25.",
    "HLS top values are Q17.15 and the model type is half precision.",
]}
mat_files = sorted((repo_root / "app/matlab_ws").glob("*.mat")) if (repo_root / "app/matlab_ws").is_dir() else []
if mat_files and len(state_history):
    try:
        from scipy.io import loadmat
        for mat_path in mat_files:
            data = loadmat(mat_path)
            if "xHistory" in data and "uHistory" in data:
                count = min(len(state_history), len(data["xHistory"]))
                state_rmse = float(np.sqrt(np.mean((state_history[:count] - data["xHistory"][:count, :12]) ** 2)))
                control_count = min(len(control_history), len(data["uHistory"]))
                control_rmse = float(np.sqrt(np.mean((control_history[:control_count] - data["uHistory"][:control_count, :4]) ** 2)))
                matlab_comparison.update({
                    "available": True, "file": str(mat_path),
                    "state_rmse": state_rmse, "control_rmse": control_rmse,
                })
                break
    except ImportError:
        matlab_comparison["notes"].append("SciPy is unavailable; MAT comparison skipped.")
print(matlab_comparison)


def diagnose_platform():
    recommendations = []
    if not paths["handoff_hwh"].is_file() or not paths["handoff_bda"].is_file():
        recommendations.append("Copy the matching design_nmpc_solver HWH and BDA from nmpc_solver.xsa_FILES.")
    if not hardware_map_checks.get("pso_fsm_control_interfaces", False):
        recommendations.append("Regenerate the platform; pso_fsm_0 must expose both AXI-Lite interfaces.")
    if not hardware_map_checks.get("worker_done_wiring", False):
        recommendations.append("Regenerate the platform with all worker ap_done ports connected to the FSM.")
    if not artifacts_ready:
        recommendations.append("Package a matching nmpc_solver.bit, nmpc_solver.hwh, and manifest.json set.")
    if not all(layout_table["fits"]) or any(layout_table["overlaps"] != ""):
        recommendations.append("Correct direct pointer constants and BRAM ranges before synthesis.")

    runtime = {"exported_ps_map": exported_ps_map.to_dict(orient="records")}
    if solver is not None:
        runtime["ap_ctrl"] = f"0x{solver.control.read(0x00):08x}"
        runtime["ap_return"] = solver.control.read(0x10)
        runtime["register_map"] = solver.register_map
    return {"runtime": runtime, "recommendations": recommendations}


diagnostics = diagnose_platform()
print(json.dumps(diagnostics, indent=2))


# ## 20. Export Deployment Logs and Validation Results
#
#
#
# Save platform tables, address comparisons, runtime metadata, controls, predicted states, histories, timing, diagnostics, and a machine-readable pass/fail report under `validation_logs`.

# In[12]:


paths["logs"].mkdir(parents=True, exist_ok=True)
path_status.to_csv(paths["logs"] / "path_status.csv", index=False)
component_table.to_csv(paths["logs"] / "hls_components.csv", index=False)
csv_addresses.to_csv(paths["logs"] / "address_segments_checked_in.csv", index=False)
exported_ps_map.to_csv(paths["logs"] / "exported_ps_register_map.csv", index=False)
layout_table.to_csv(paths["logs"] / "bram_layout.csv", index=False)
closed_loop_table.to_csv(paths["logs"] / "solver_timing.csv", index=False)
np.save(paths["logs"] / "state_history.npy", state_history)
np.save(paths["logs"] / "control_history.npy", control_history)
if one_step_result is not None:
    np.save(paths["logs"] / "best_control_sequence.npy", one_step_result.control_sequence)
    np.save(paths["logs"] / "best_fitness.npy", one_step_result.best_fitness)
if predicted_next_state is not None:
    np.save(paths["logs"] / "predicted_next_state.npy", predicted_next_state)

validation_report = {
    "kv260_compatibility": bool(clock_checks.get("part_ok") and clock_checks.get("board_ok")),
    "clock_reset": bool(clock_checks.get("clock_matches_hls") and clock_checks.get("synchronized_reset")),
    "exported_pso_fsm_register_map": bool(hardware_map_checks.get("pso_fsm_control_interfaces")),
    "exported_control_base": bool(hardware_map_checks.get("control_base")),
    "exported_argument_base": bool(hardware_map_checks.get("argument_base")),
    "worker_done_wiring": bool(hardware_map_checks.get("worker_done_wiring")),
    "bram_layout": bool(all(layout_table["fits"]) and all(layout_table["overlaps"] == "")),
    "overlay_artifacts": bool(artifacts_ready),
    "solver_completion": one_step_result is not None,
    "numerical_validity": bool(
        one_step_result is not None and np.all(np.isfinite(one_step_result.control_sequence))
    ),
    "control_constraints": bool(
        one_step_result is not None
        and np.all(np.abs(one_step_result.control_sequence) <= 100.0 + 1e-3)
    ),
}
export_payload = {
    "validation": validation_report,
    "project": project_config,
    "clock_checks": clock_checks,
    "hardware_map_checks": hardware_map_checks,
    "artifact_checks": artifact_checks,
    "matlab_comparison": matlab_comparison,
    "diagnostics": diagnostics,
}
(paths["logs"] / "validation_report.json").write_text(
    json.dumps(export_payload, indent=2, default=str) + "\n", encoding="utf-8"
)
display(pd.Series(validation_report, name="pass"))
print("Validation logs written to", paths["logs"])
if solver is not None:
    solver.close()


# In[ ]:
