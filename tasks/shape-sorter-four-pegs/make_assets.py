"""Write the task's USD parts from the FMB peg-board meshes (OBJ, metres, Z up).

    python make_assets.py <dir with the FMB OBJ files> <out dir>

Each part becomes one rigid body in its own .usda: pegs collide as their convex hull (all four
shapes used here are convex), the board as a signed distance field so its holes are real
geometry with the printed 1.5 mm clearance per side. Pegs are PLA (1.24 g/cm3); the board is
kinematic (clamped to the table). Pure Python, no pxr needed.
"""

from __future__ import annotations

import sys
from pathlib import Path

PLA_DENSITY = 1240.0  # kg/m3
PEG_COLOR = (0.93, 0.93, 0.90)  # off-white PLA
BOARD_COLOR = (0.20, 0.42, 0.75)  # blue PLA

PARTS = {
    # name: (obj stem, kind)
    "board": ("board_medium", "board"),
    "peg_rectangle": ("peg_rectangle_medium_short", "peg"),
    "peg_rectangle_long": ("peg_rectangle_medium_long", "peg"),
    "peg_hexagon": ("peg_hexagon_medium_short", "peg"),
    "peg_arch": ("peg_arch_medium_short", "peg"),
    "peg_round": ("peg_round_medium_short", "peg"),
    "peg_oval": ("peg_oval_medium_short", "peg"),
}


def read_obj(path: Path):
    verts, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append(tuple(float(x) for x in line.split()[1:4]))
        elif line.startswith("f "):
            faces.append([int(tok.split("/")[0]) - 1 for tok in line.split()[1:]])
    return verts, faces


def volume(verts, faces) -> float:
    vol = 0.0
    for f in faces:
        a = verts[f[0]]
        for i in range(1, len(f) - 1):
            b, c = verts[f[i]], verts[f[i + 1]]
            vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                    + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return abs(vol)


def write_usda(name: str, kind: str, verts, faces, out: Path) -> dict:
    lo = [min(v[i] for v in verts) for i in range(3)]
    hi = [max(v[i] for v in verts) for i in range(3)]
    vol = volume(verts, faces)
    pts = ", ".join(f"({x:.6f}, {y:.6f}, {z:.6f})" for x, y, z in verts)
    counts = ", ".join(str(len(f)) for f in faces)
    idx = ", ".join(str(i) for f in faces for i in f)
    if kind == "peg":
        mass = vol * PLA_DENSITY
        body = (f"        float physics:mass = {mass:.5f}\n"
                "        int physxRigidBody:solverPositionIterationCount = 32\n"
                "        int physxRigidBody:solverVelocityIterationCount = 4\n")
        coll_api = '"PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI"'
        coll = '        uniform token physics:approximation = "convexHull"\n'
        color = PEG_COLOR
        friction = (0.6, 0.5)  # PLA on rubber pads is set by the pads; PLA on PLA ~0.5
    else:
        mass = None
        body = "        bool physics:kinematicEnabled = 1\n"
        coll_api = '"PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI", "PhysxSDFMeshCollisionAPI"'
        coll = ('        uniform token physics:approximation = "sdf"\n'
                "        int physxSDFMeshCollision:sdfResolution = 384\n")
        color = BOARD_COLOR
        friction = (0.4, 0.35)
    # Contact offsets well under the 1.5 mm clearance per side, so a peg can enter its hole (as in the accepted
    # battery-pack-assembly task's holder).
    coll += ("        float physxCollision:contactOffset = 0.0004\n"
             "        float physxCollision:restOffset = 0\n")
    text = f'''#usda 1.0
(
    doc = "{name}: from the FMB peg-board CAD ({kind}), CC BY 4.0, functional-manipulation-benchmark.github.io"
    defaultPrim = "{name}"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{name}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
)
{{
{body}
    def Material "pla" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {friction[0]}
        float physics:dynamicFriction = {friction[1]}
        float physics:restitution = 0
    }}

    def Mesh "mesh" (
        prepend apiSchemas = [{coll_api}, "MaterialBindingAPI"]
    )
    {{
{coll}        int[] faceVertexCounts = [{counts}]
        int[] faceVertexIndices = [{idx}]
        point3f[] points = [{pts}]
        float3[] extent = [({lo[0]:.6f}, {lo[1]:.6f}, {lo[2]:.6f}), ({hi[0]:.6f}, {hi[1]:.6f}, {hi[2]:.6f})]
        color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})]
        uniform token subdivisionScheme = "none"
        rel material:binding:physics = </{name}/pla> (
            bindMaterialAs = "strongerThanDescendants"
        )
    }}
}}
'''
    out.write_text(text)
    return {"name": name, "volume_cm3": vol * 1e6, "mass_kg": mass, "lo": lo, "hi": hi}


def main(src: str, dst: str) -> None:
    src_dir, out_dir = Path(src), Path(dst)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name, (stem, kind) in PARTS.items():
        verts, faces = read_obj(src_dir / f"{stem}.obj")
        info = write_usda(name, kind, verts, faces, out_dir / f"{name}.usda")
        print(info)


if __name__ == "__main__":
    main(*sys.argv[1:3])
