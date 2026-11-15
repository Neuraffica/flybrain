"""Offline Bittle renderer. See NOTICE in the generated asset directory.

Run with bpy 5.0.1 and Pillow; source is the pinned Petoi ros1 model.
No renderer or mesh is needed by either application at runtime.
"""

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import bpy
from mathutils import Vector

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "work/bittle/model"
OUTPUT = ROOT / "work/bittle/renders"
OUTPUT.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action="SELECT")
bpy.ops.object.delete(use_global=False)


def material(name, color, roughness=0.35, metallic=0):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get("Principled BSDF")
    shader.inputs["Base Color"].default_value = (*color, 1)
    shader.inputs["Roughness"].default_value = roughness
    shader.inputs["Metallic"].default_value = metallic
    return mat


yellow = material("injection molded yellow", (0.95, 0.55, 0.012))
black = material("black polymer", (0.018, 0.022, 0.026), 0.3)
metal = material("screw steel", (0.35, 0.38, 0.41), 0.23, 0.8)
white = material("IR board", (0.78, 0.8, 0.74), 0.45)
lens = material("IR lenses", (0.008, 0.012, 0.019), 0.13)
xml = ET.parse(SOURCE / "urdf/bittle.xacro").getroot()
links = {}
meshes = {}
for link in xml.findall("link"):
    name = link.get("name")
    empty = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(empty)
    links[name] = empty
    visual = link.find("visual")
    if visual is None:
        continue
    mesh = visual.find("geometry/mesh")
    bpy.ops.wm.stl_import(filepath=str(SOURCE / "meshes" / Path(mesh.get("filename")).name))
    obj = bpy.context.object
    obj.name = name + "-mesh"
    obj.parent = empty
    obj.scale = (0.1, 0.1, 0.1)
    obj.location = [float(v) * 100 for v in visual.find("origin").get("xyz").split()]
    obj.rotation_euler = [float(v) for v in visual.find("origin").get("rpy").split()]
    mat = black
    if name in ("base_link", "front__1", "rear__1", "head__1"):
        mat = yellow
    elif name.startswith(("screw", "logo_")):
        mat = metal
    obj.data.materials.append(mat)
    # Imported surface normals retain the manufactured hard edges.
    bevel = obj.modifiers.new("submillimeter edge highlights", "BEVEL")
    bevel.width = 0.35
    bevel.segments = 2
    bevel.limit_method = "ANGLE"
    meshes[name] = obj

joints = {}
for joint in xml.findall("joint"):
    obj = links[joint.find("child").get("link")]
    obj.parent = links[joint.find("parent").get("link")]
    obj.location = [float(v) * 100 for v in joint.find("origin").get("xyz").split()]
    if joint.get("type") != "fixed":
        axis = Vector([float(v) for v in joint.find("axis").get("xyz").split()])
        obj.rotation_mode = "AXIS_ANGLE"
        joints[joint.get("name")] = (obj, axis)

bpy.context.view_layer.update()
# Original accessory geometry: a small bone-shaped double IR PCB on the head.
sensor_parent = links["servo_neck__1"]


def accessory(name, loc, scale, mat, sphere=False):
    if sphere:
        bpy.ops.mesh.primitive_uv_sphere_add(segments=24, ring_count=12)
    else:
        bpy.ops.mesh.primitive_cube_add()
    obj = bpy.context.object
    obj.name = name
    obj.location = loc
    obj.scale = scale
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    obj.data.materials.append(mat)
    if not sphere:
        mod = obj.modifiers.new("rounded PCB edges", "BEVEL")
        mod.width = 0.12
        mod.segments = 4
    matrix = obj.matrix_world.copy()
    obj.parent = sensor_parent
    obj.matrix_world = matrix
    return obj


accessory("IR bridge", (8.3, 0, 6.8), (0.55, 1.65, 0.14), white)
for y in (-1.65, 1.65):
    accessory("IR round end", (8.3, y, 6.8), (0.6, 0.6, 0.16), white, True)
    accessory("forward IR aperture", (8.85, y, 6.85), (0.18, 0.32, 0.26), lens, True)

scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.samples = 32
scene.cycles.use_denoising = True
scene.render.resolution_x = 384
scene.render.resolution_y = 384
scene.render.resolution_percentage = 100
scene.render.film_transparent = True
scene.render.image_settings.file_format = "PNG"
scene.render.image_settings.color_mode = "RGBA"
scene.world.color = (0.22, 0.22, 0.22)
scene.view_settings.view_transform = "AgX"

for name, loc, power, size in [
    ("key", (10, -15, 30), 8000, 18),
    ("fill", (-10, 15, 20), 6500, 15),
    ("rim", (-20, -5, 8), 4000, 12),
]:
    data = bpy.data.lights.new(name, "AREA")
    data.energy = power
    data.shape = "DISK"
    data.size = size
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.location = loc
    obj.rotation_euler = (-obj.location).to_track_quat("-Z", "Y").to_euler()

camera_data = bpy.data.cameras.new("orthographic")
camera = bpy.data.objects.new("orthographic", camera_data)
bpy.context.collection.objects.link(camera)
scene.camera = camera
camera_data.type = "ORTHO"
camera_data.ortho_scale = 32


def pose(phase=None, rest=False):
    for name, (obj, axis) in joints.items():
        a = 0
        if name != "neck_joint":
            rear = "rs_" in name or "rt_" in name
            left = name.startswith("shl")
            swing = (
                0
                if phase is None
                else math.sin(2 * math.pi * (phase + (0.5 if rear != left else 0)))
            )
            if name.endswith("s_joint"):
                a = (0 if rest else -0.8 + 0.25 * swing) * (-1 if rear else 1)
            else:
                a = 0 if rest else 1.5 - 0.3 * swing
        obj.rotation_axis_angle = (a, *axis)
    bpy.context.view_layer.update()


def render(name, view):
    if view == "top":
        camera.location = (0, 0, 60)
        camera.rotation_euler = (0, 0, -math.pi / 2)
    else:
        floor = min(
            (obj.matrix_world @ Vector(corner)).z
            for name, obj in meshes.items()
            if name.startswith("shank")
            for corner in obj.bound_box
        )
        camera.location = (0, -60, floor + 14)
        camera.rotation_euler = (
            (Vector((0, 0, floor + 14)) - camera.location).to_track_quat("-Z", "Y").to_euler()
        )
    scene.render.filepath = str(OUTPUT / (name + ".png"))
    bpy.ops.render.render(write_still=True)


pose()
render("preview-top", "top")
render("preview-side", "side")
bpy.ops.wm.save_as_mainfile(filepath=str(OUTPUT / "bittle.blend"))
if not os.environ.get("BITTLE_PREVIEW"):
    head_objects = [sensor_parent, *sensor_parent.children_recursive]
    renderables = [obj for obj in scene.objects if obj.type == "MESH"]
    for obj in renderables:
        obj.hide_render = obj not in head_objects
    render("head", "top")
    for view in ("top", "side"):
        for obj in renderables:
            obj.hide_render = view == "top" and obj in head_objects
        for i in range(10):
            pose(None if i < 2 else (i - 2) / 8, rest=i == 1)
            render(f"{view}-{i}", view)
    from PIL import Image

    assets = ROOT / "packages/flybrain-demo-api/src/flybrain_demo_api/static/bittle"
    for view in ("top", "side"):
        atlas = Image.new("RGBA", (3840, 384))
        for i in range(10):
            atlas.paste(Image.open(OUTPUT / f"{view}-{i}.png"), (384 * i, 0))
        atlas.save(assets / f"{view}.webp", quality=88, method=6)
    Image.open(OUTPUT / "head.png").save(assets / "head.webp", quality=90, method=6)
