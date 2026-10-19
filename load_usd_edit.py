"""Load a USD(.usda) file into MuJoCo from Python and edit joint stiffness/damping.

Usage:
    python load_usd_edit.py /path/to/file.usda [--view]

Loading USD needs the USD decoder plugin (libusd_decoder_plugin.so) to be
registered in this Python process. See the setup notes for how to do that.
"""
import argparse
import os
import sys

import mujoco


def load_spec(path):
    """Try loading the USD file directly; fall back to a wrapper MJCF."""
    # Attempt 1: open the USD directly (the decoder handles the extension).
    try:
        spec = mujoco.MjSpec.from_file(path)
        print("Loaded directly with MjSpec.from_file")
        return spec
    except Exception as e:  # noqa: BLE001 - we want to report and continue
        print(f"Direct load failed: {e}")

    # Attempt 2: reference it from MJCF as documented for USD assets.
    # NOTE: the docs show the <model> asset tag; how to attach it into the
    # world may need adjusting for your file (unverified).
    xml = f"""
<mujoco>
  <asset>
    <model file="{os.path.abspath(path)}" name="usd_model" content_type="text/usd"/>
  </asset>
  <worldbody>
    <frame>
      <attach model="usd_model" body="world" prefix="usd_"/>
    </frame>
  </worldbody>
</mujoco>
"""
    spec = mujoco.MjSpec.from_string(xml)
    print("Loaded via wrapper MJCF")
    return spec


def describe(model):
    print(f"\nnbody={model.nbody} njnt={model.njnt} ngeom={model.ngeom} nq={model.nq} nv={model.nv}")
    print(f"{'joint':32s} {'type':8s} {'stiffness':>10s} {'damping':>10s}")
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j) or f"joint_{j}"
        jtype = mujoco.mjtJoint(model.jnt_type[j]).name.replace("mjJNT_", "")
        dof = model.jnt_dofadr[j]
        print(f"{name:32s} {jtype:8s} {model.jnt_stiffness[j]:10.4g} {model.dof_damping[dof]:10.4g}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("usd_path")
    ap.add_argument("--view", action="store_true", help="open the interactive viewer")
    args = ap.parse_args()

    spec = load_spec(args.usd_path)

    # --- Option 1: edit on the spec, before compiling (cleanest) -----------
    for joint in spec.joints:
        print(f"spec joint {joint.name!r}: stiffness={joint.stiffness}, damping={joint.damping}")
        joint.stiffness = 5.0   # EDIT ME
        joint.damping = 0.5     # EDIT ME

    model = spec.compile()
    data = mujoco.MjData(model)
    describe(model)

    # --- Option 2: edit the compiled model directly (e.g. to sweep values) --
    # for j in range(model.njnt):
    #     model.jnt_stiffness[j] = 10.0
    #     model.dof_damping[model.jnt_dofadr[j]] = 1.0   # first DoF of this joint

    if args.view:
        from mujoco import viewer
        viewer.launch(model, data)
    else:
        for _ in range(1000):
            mujoco.mj_step(model, data)
        print(f"\nstepped 1000 times, sim time = {data.time:.3f}s")


if __name__ == "__main__":
    sys.exit(main())