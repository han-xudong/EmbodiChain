# ----------------------------------------------------------------------------
# Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ----------------------------------------------------------------------------

"""Asset source and cache identity for CobotMagicArm."""

from __future__ import annotations

import os
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET

import open3d as o3d
from embodichain.data.dataset import EmbodiChainDataset
from embodichain.data.constants import (
    EMBODICHAIN_DOWNLOAD_PREFIX,
    EMBODICHAIN_DEFAULT_DATA_ROOT,
)

from .._arm_assets import _write_atomic

__all__ = ["CobotMagicArm"]

robot_assets = "robot_assets"

_COMPETITION_POSITION_LIMITS = {
    "joint5": ("-1.3", "1.3"),
    "joint6": ("-2.988", "2.988"),
}


def _competition_urdf(source_path: str) -> str:
    """Resolve the RoboSynChallenge arm range using the V100 asset geometry."""
    source = Path(source_path).resolve()
    arm = ET.parse(source).getroot()
    for name, (lower, upper) in _COMPETITION_POSITION_LIMITS.items():
        limit = arm.find(f"joint[@name='{name}']/limit")
        if limit is None:
            raise ValueError(f"CobotMagic source has no position limits for {name}.")
        limit.set("lower", lower)
        limit.set("upper", upper)

    for mesh in arm.iter("mesh"):
        filename = mesh.get("filename")
        if not filename or Path(filename).is_absolute():
            continue
        path = source.parent / filename
        if not path.is_file() and filename.startswith("../"):
            path = source.parent / filename.removeprefix("../")
        mesh.set("filename", str(path.resolve()))

    contents = ET.tostring(arm, encoding="utf-8", xml_declaration=True)
    digest = sha256(contents).hexdigest()[:16]
    destination = (
        Path(EMBODICHAIN_DEFAULT_DATA_ROOT)
        / "generated"
        / "CobotMagicArm"
        / f"competition-{digest}.urdf"
    )
    if not destination.is_file() or destination.read_bytes() != contents:
        _write_atomic(destination, contents)
    return str(destination)


class CobotMagicArm(EmbodiChainDataset):
    """Dataset class for the Cobot Magic Arm robot.

    Reference:
        https://global.agilex.ai/products/cobot-magic

    Directory structure:
        CobotMagicArm/
            CobotMagicNoGripper.urdf
            CobotMagicWithGripperV70.urdf
            CobotMagicWithGripperV70NewUV.urdf
            CobotMagicWithGripperV70NoMaterial.urdf
            CobotMagicWithGripperV100.urdf
            CobotMagicWithGripperV100NewUV.urdf
            CobotMagicWithGripperV100NoMaterial.urdf

    Example usage:
        >>> from embodichain.data.robot_dataset import CobotMagicArm
        >>> dataset = CobotMagicArm()
        or
        >>> from embodichain.data import get_data_path
        >>> print(get_data_path("CobotMagicArm/CobotMagicWithGripperV100.urdf"))
    """

    def __init__(self, data_root: str = None):
        data_descriptor = o3d.data.DataDescriptor(
            os.path.join(
                EMBODICHAIN_DOWNLOAD_PREFIX, robot_assets, "CobotMagicArmV4.zip"
            ),
            "8cc54c240c2f26e84e22250c8364b0ec",
        )
        prefix = type(self).__name__
        path = EMBODICHAIN_DEFAULT_DATA_ROOT if data_root is None else data_root

        super().__init__(prefix, data_descriptor, path)
