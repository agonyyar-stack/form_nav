#!/usr/bin/env python3
"""校验 formations.yaml 中的领导者三角形与重心编队权重。"""

import argparse
import math
import sys

import yaml


def parse_agent_id(key):
    text = str(key)
    if text.startswith("uav"):
        text = text[3:]
    try:
        agent_id = int(text)
    except ValueError as exc:
        raise ValueError(f"非法无人机编号: {key}") from exc
    if agent_id < 0:
        raise ValueError(f"非法无人机编号: {key}")
    return agent_id


def determinant3(matrix):
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def solve3(matrix, vector):
    det = determinant3(matrix)
    if abs(det) < 1e-9:
        raise ValueError("领导者三角形退化")

    result = []
    for column in range(3):
        replaced = [row[:] for row in matrix]
        for row in range(3):
            replaced[row][column] = vector[row]
        result.append(determinant3(replaced) / det)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "config_file",
        nargs="?",
        default="config/formations.yaml",
        help="待检查的 formations.yaml",
    )
    args = parser.parse_args()

    with open(args.config_file, "r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if config.get("formation_mode", "offset") != "barycentric":
        raise ValueError("当前配置不是 barycentric 模式")

    offsets = {
        parse_agent_id(key): [float(value) for value in offset]
        for key, offset in config["offsets"].items()
    }
    leaders = [int(value) for value in config["leaders"]]
    if len(leaders) != 3 or len(set(leaders)) != 3:
        raise ValueError("barycentric 模式必须配置三个不同的领导者")

    triangle = [
        [offsets[leaders[0]][0], offsets[leaders[1]][0], offsets[leaders[2]][0]],
        [offsets[leaders[0]][1], offsets[leaders[1]][1], offsets[leaders[2]][1]],
        [1.0, 1.0, 1.0],
    ]
    signed_area = 0.5 * (
        (offsets[leaders[1]][0] - offsets[leaders[0]][0])
        * (offsets[leaders[2]][1] - offsets[leaders[0]][1])
        - (offsets[leaders[1]][1] - offsets[leaders[0]][1])
        * (offsets[leaders[2]][0] - offsets[leaders[0]][0])
    )
    if abs(signed_area) < 1e-6:
        raise ValueError("领导者三角形在 xy 平面退化")

    configured_weights = config.get("weights", {})
    followers = [
        agent_id
        for agent_id in offsets
        if agent_id not in set(leaders)
    ]

    print(f"leaders={leaders}")
    print(f"signed_area={signed_area:.6f}")

    for follower_id in sorted(followers):
        offset = offsets[follower_id]
        expected = solve3(triangle, [offset[0], offset[1], 1.0])
        row = configured_weights.get(f"uav{follower_id}", {})
        configured = [float(row.get(f"uav{leader_id}", math.nan)) for leader_id in leaders]

        if any(not math.isfinite(value) or value < 0.0 for value in configured):
            raise ValueError(f"uav{follower_id}: 权重缺失、非有限或为负")
        if abs(sum(configured) - 1.0) > 1e-6:
            raise ValueError(f"uav{follower_id}: 权重和不为 1")
        if max(abs(a - b) for a, b in zip(expected, configured)) > 1e-5:
            raise ValueError(f"uav{follower_id}: 配置权重与标称位置不一致")

        reconstructed = [
            sum(configured[index] * offsets[leaders[index]][axis] for index in range(3))
            for axis in range(3)
        ]
        reconstruction_error = max(
            abs(reconstructed[axis] - offset[axis]) for axis in range(3)
        )
        if reconstruction_error > 1e-4:
            raise ValueError(f"uav{follower_id}: 标称位置重构误差过大")

        formatted = ", ".join(
            f"uav{leader_id}={weight:.6f}"
            for leader_id, weight in zip(leaders, configured)
        )
        print(
            f"uav{follower_id}: {formatted}, "
            f"max_position_error={reconstruction_error:.3e}"
        )

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyError, TypeError, ValueError, OSError) as error:
        print(f"barycentric formation check failed: {error}", file=sys.stderr)
        sys.exit(1)
