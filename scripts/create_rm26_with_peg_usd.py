#!/usr/bin/env python3
"""Create an overlay USD that attaches the peg tube to right_end_link."""
from __future__ import annotations

import argparse
from pathlib import Path


def write_overlay(path: Path, base_usd: str, peg_usd: str, offset_z: float):
    content = f"""#usda 1.0
(
    defaultPrim = \"Robot\"
)

over \"Robot\" (
    prepend references = @{base_usd}@
)
{{
    over \"right_end_link\"
    {{
        def Xform \"peg_tube\"
        {{
            double3 xformOp:translate = (0, 0, {offset_z:.6f})
            uniform token[] xformOpOrder = [\"xformOp:translate\"]
            def Xform \"mesh\" (
                prepend references = @{peg_usd}@
            )
            {{
            }}
        }}
    }}
}}
"""
    path.write_text(content)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=None)
    parser.add_argument("--base", default=None)
    parser.add_argument("--peg", default=None)
    parser.add_argument("--offset_z", type=float, default=0.075)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    custom_dir = repo_root / "isaacLab" / "manipulation" / "assets" / "usd" / "custom"
    default_out = custom_dir / "rm26_version2_engineering_model_with_peg.usda"
    base_usd = args.base or "../rm26_version2_engineering_model/rm26_version2_engineering_model.usd"
    peg_usd = args.peg or "./peg_tube.usda"

    out_path = Path(args.output) if args.output else default_out
    out_path.parent.mkdir(parents=True, exist_ok=True)

    write_overlay(out_path, base_usd, peg_usd, args.offset_z)
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
