"""Python software-in-the-loop (SIL) MPC for the Husky A300 in Gazebo.

The core (pso, reference, controller, metrics, gz_truth parsing) is plain Python + numpy and is
tested offline. Only sil_node.py imports rclpy, inside its main().
"""

from pathlib import Path
import sys

_HUSKY = str(Path(__file__).resolve().parents[1])
if _HUSKY not in sys.path:
    sys.path.insert(0, _HUSKY)  # unicycle_model, command_ownership, identify_effective_track
