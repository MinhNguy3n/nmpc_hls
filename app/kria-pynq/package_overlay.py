from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import zipfile


DESIGN_REVISION = 3
BITSTREAM_MEMBERS = (
    "design_nmpc_solver.bit"
)
HWH_MEMBERS = (
    "design_nmpc_solver.hwh"
)
BDA_MEMBERS = (
    "design_nmpc_solver.bda"
)
REQUIRED_HWH_TOKENS = (
    'FULLNAME="/nmpc_solver/pso_fsm_0"',
    "s_axi_control",
    "s_axi_control_r",
    "init_s_ap_done",
    "eval_s_ap_done",
    "detect_min_ap_done",
    "update_s_ap_done",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def choose_member(
    names: list[str],
    suffix: str,
    preferred: tuple[str, ...],
    avoid_tokens: tuple[str, ...] = (),
) -> str:
    matches = list(dict.fromkeys(name for name in names if name.lower().endswith(suffix)))
    if not matches:
        raise RuntimeError(f"No {suffix} member found in XSA")

    def pick_best(candidates: list[str]) -> str:
        return sorted(candidates, key=lambda name: (len(Path(name).parts), len(name), name))[0]

    for preferred_name in preferred:
        preferred_matches = [name for name in matches if Path(name).name == preferred_name]
        if preferred_matches:
            return pick_best(preferred_matches)

    candidates = matches
    if avoid_tokens:
        filtered = [
            name
            for name in matches
            if not any(token in Path(name).name for token in avoid_tokens)
        ]
        if filtered:
            candidates = filtered

    top_level = [name for name in candidates if len(Path(name).parts) == 1]
    if top_level:
        return pick_best(top_level)

    return pick_best(candidates)


def pso_fsm_register_map(bda_data: bytes) -> dict[str, dict[str, int]]:
    bda = json.loads(bda_data)
    interfaces: dict[str, dict[str, int]] = {}
    for vertex in bda["graphjs"]["vertices"].values():
        if (
            vertex.get("TU") == "register"
            and vertex.get("SX") == "/nmpc_solver/pso_fsm_0"
            and vertex.get("SI") in {"s_axi_control", "s_axi_control_r"}
        ):
            interfaces[vertex["SI"]] = {
                "base": int(vertex["BA"], 0),
                "range": int(vertex["HA"], 0) - int(vertex["BA"], 0) + 1,
            }

    required_interfaces = {"s_axi_control", "s_axi_control_r"}
    if interfaces.keys() != required_interfaces:
        raise RuntimeError(
            "XSA BDA does not expose the required pso_fsm_0 control interfaces; "
            f"found={sorted(interfaces)}"
        )
    return interfaces


def package_overlay(xsa_path: Path, output_dir: Path) -> None:
    if not xsa_path.is_file():
        raise FileNotFoundError(xsa_path)
    output_dir.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(xsa_path) as archive:
        names = archive.namelist()
        bit_member = choose_member(names, ".bit", BITSTREAM_MEMBERS)
        hwh_member = choose_member(
            names,
            ".hwh",
            HWH_MEMBERS,
            avoid_tokens=("smartconnect", "axi_smc"),
        )
        bda_member = choose_member(names, ".bda", BDA_MEMBERS)
        hwh_data = archive.read(hwh_member)
        hwh_text = hwh_data.decode("utf-8", errors="replace")
        missing = [token for token in REQUIRED_HWH_TOKENS if token not in hwh_text]
        if missing:
            raise RuntimeError(
                "XSA does not contain the required NMPC runtime interfaces; "
                f"missing={missing}. Rebuild with "
                "hw/build_kv260_overlay.sh."
            )
        register_map = pso_fsm_register_map(archive.read(bda_member))

        bit_path = output_dir / "nmpc_solver.bit"
        hwh_path = output_dir / "nmpc_solver.hwh"
        bit_path.write_bytes(archive.read(bit_member))
        hwh_path.write_bytes(hwh_data)

    manifest = {
        "design_revision": DESIGN_REVISION,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_xsa": str(xsa_path.resolve()),
        "platform": {
            "hwh_member": hwh_member,
            "bda_member": bda_member,
            "pso_fsm_0": register_map,
        },
        "files": {
            bit_path.name: {"sha256": sha256(bit_path)},
            hwh_path.name: {"sha256": sha256(hwh_path)},
        },
    }
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract and validate a PYNQ overlay from the KV260 XSA"
    )
    parser.add_argument("--xsa", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    package_overlay(arguments.xsa, arguments.output)


if __name__ == "__main__":
    main()
