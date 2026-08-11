import os
import colorsys
import xml.etree.ElementTree as ET
import random
import math
import numpy as np
from env.utils.xml_utils import resolve_includes, resolve_all_file_paths
from env.utils.scene_clearance_bounds import load_obj_vertices
from env.utils.sorting_scene import validate_sorting_layout
from env.utils.ycb_assets import resolve_ycb_assets


SCENE_MODES = ("mix", "random", "assign")
SCENE_OBJECT_COUNT = 6
RANDOM_SCENE_CATEGORY_ORDER = (
    "cylindrical_can",
    "banana",
    "round_top",
    "box",
    "tool_top",
)
MESH_SPAWN_CLEARANCE_M = 0.0005


def remove_elements_by_name(root, body_names_to_remove):
    """Remove body elements by name more efficiently using parent map."""
    # Create a parent map
    parent_map = {c: p for p in root.iter() for c in p}

    for body_name in body_names_to_remove:
        for body in root.findall(f".//body[@name='{body_name}']"):
            parent = parent_map.get(body)
            if parent is not None:
                print(f"[INFO] Removing {body_name} from scene")
                parent.remove(body)

def configure_cameras_in_scene(root, camera_names=[]):
    """ Configure cameras in the Mujoco scene.

    if a camera name is not in the camera_names list, some bodies will be removed.

    """

    bodies_to_remove = []

    if "camera_orbbec" not in camera_names:
        bodies_to_remove.extend(['camera_mount_pt2', 'camera_mount_orbbec'])

    if "camera_orbbec_static" not in camera_names:
        bodies_to_remove.append('camera_mount_orbbec_static')

    if "camera_realsense" not in camera_names:
        bodies_to_remove.append('camera_mount_realsense')

    # Remove all specified bodies at once
    if bodies_to_remove:
        remove_elements_by_name(root, bodies_to_remove)



def create_object_xml(obj_config):
    """Create XML element for an object based on its configuration."""
    name = obj_config['name']
    obj_type = obj_config['type']
    size = obj_config['size']
    position = obj_config['position']
    orientation = obj_config['orientation']
    color = obj_config.get('color', [0.5, 0.5, 0.5, 1.0])
    mass = obj_config.get('mass', 0.1)
    friction = obj_config.get('friction', [0.9, 0.2, 0.05])

    # Create body element
    body = ET.Element('body', name=name, pos=f"{position[0]} {position[1]} {position[2]}",
                      quat=f"{orientation[0]} {orientation[1]} {orientation[2]} {orientation[3]}")

    # Create geom element based on type
    if obj_type == "box":
        geom = ET.SubElement(body, 'geom',
                           name=f"{name}_geom",
                           type="box",
                           size=f"{size[0]} {size[1]} {size[2]}",
                           rgba=f"{color[0]} {color[1]} {color[2]} {color[3]}",
                           friction=f"{friction[0]} {friction[1]} {friction[2]}",
                           mass=f"{mass}")
    elif obj_type == "sphere":
        geom = ET.SubElement(body, 'geom',
                           name=f"{name}_geom",
                           type="sphere",
                           size=f"{size[0]}",
                           rgba=f"{color[0]} {color[1]} {color[2]} {color[3]}",
                           friction=f"{friction[0]} {friction[1]} {friction[2]}",
                           mass=f"{mass}")
    elif obj_type == "cylinder":
        geom = ET.SubElement(body, 'geom',
                           name=f"{name}_geom",
                           type="cylinder",
                           size=f"{size[0]} {size[1]}",
                           rgba=f"{color[0]} {color[1]} {color[2]} {color[3]}",
                           friction=f"{friction[0]} {friction[1]} {friction[2]}",
                           mass=f"{mass}")
    else:
        raise ValueError(f"Unsupported object type: {obj_type}")

    # Add joint for free movement
    joint = ET.SubElement(body, 'joint', name=f"{name}_joint", type="free")

    return body


def _classification_bin_body(bin_config, center_xy, table_surface_z):
    """Build one static open-top bin from a floor and four box walls."""
    name = str(bin_config["name"])
    inner_size = np.asarray(
        bin_config.get("inner_size", [0.24, 0.20]),
        dtype=float,
    )
    wall_height = float(bin_config.get("wall_height", 0.10))
    wall_thickness = float(bin_config.get("wall_thickness", 0.01))
    floor_thickness = float(bin_config.get("floor_thickness", 0.01))
    color = np.asarray(
        bin_config.get("color", [0.3, 0.3, 0.3, 1.0]),
        dtype=float,
    )
    if inner_size.shape != (2,) or np.any(inner_size <= 0.0):
        raise ValueError(
            f"Classification bin {name!r} needs positive inner_size [x, y]."
        )
    if min(wall_height, wall_thickness, floor_thickness) <= 0.0:
        raise ValueError(
            f"Classification bin {name!r} dimensions must be positive."
        )
    if color.shape != (4,) or np.any((color < 0.0) | (color > 1.0)):
        raise ValueError(
            f"Classification bin {name!r} color must be four values in [0, 1]."
        )

    half_x, half_y = inner_size / 2.0
    body = ET.Element(
        "body",
        name=name,
        pos=f"{center_xy[0]} {center_xy[1]} {table_surface_z}",
    )

    def add_geom(suffix, pos, size):
        ET.SubElement(
            body,
            "geom",
            name=f"{name}_{suffix}",
            type="box",
            pos=" ".join(str(value) for value in pos),
            size=" ".join(str(value) for value in size),
            rgba=" ".join(str(value) for value in color),
            friction="0.9 0.2 0.05",
            contype="1",
            conaffinity="1",
        )

    add_geom(
        "floor",
        [0.0, 0.0, floor_thickness / 2.0],
        [half_x + wall_thickness, half_y + wall_thickness,
         floor_thickness / 2.0],
    )
    wall_z = floor_thickness + wall_height / 2.0
    add_geom(
        "wall_x_pos",
        [half_x + wall_thickness / 2.0, 0.0, wall_z],
        [wall_thickness / 2.0, half_y + wall_thickness,
         wall_height / 2.0],
    )
    add_geom(
        "wall_x_neg",
        [-half_x - wall_thickness / 2.0, 0.0, wall_z],
        [wall_thickness / 2.0, half_y + wall_thickness,
         wall_height / 2.0],
    )
    add_geom(
        "wall_y_pos",
        [0.0, half_y + wall_thickness / 2.0, wall_z],
        [half_x, wall_thickness / 2.0, wall_height / 2.0],
    )
    add_geom(
        "wall_y_neg",
        [0.0, -half_y - wall_thickness / 2.0, wall_z],
        [half_x, wall_thickness / 2.0, wall_height / 2.0],
    )
    return body


def add_classification_bins(
    worldbody,
    classification_bins,
    table_surface_z=-0.025,
    rng=random,
):
    """Add the configured randomized food bin to the canonical scene."""
    if not classification_bins or not classification_bins.get("enabled", False):
        return None
    bins = list(classification_bins.get("bins", []))
    if len(bins) != 1 or str(bins[0].get("category", "")) != "food":
        raise ValueError(
            "Sorting mode requires exactly one classification bin for 'food'."
        )
    position_range = classification_bins.get("position_range", {})
    x_range = np.asarray(position_range.get("x", ()), dtype=float)
    y_range = np.asarray(position_range.get("y", ()), dtype=float)
    if (
        x_range.shape != (2,)
        or y_range.shape != (2,)
        or x_range[0] > x_range[1]
        or y_range[0] > y_range[1]
    ):
        raise ValueError(
            "classification_bins.position_range needs ordered x/y pairs."
        )

    bin_config = dict(bins[0])
    bin_config.setdefault("color", [0.3, 0.3, 0.3, 1.0])
    center_xy = [
        rng.uniform(float(x_range[0]), float(x_range[1])),
        rng.uniform(float(y_range[0]), float(y_range[1])),
    ]
    if bool(classification_bins.get("random_color", False)):
        rgb = colorsys.hsv_to_rgb(
            rng.uniform(0.0, 1.0),
            rng.uniform(0.75, 1.0),
            rng.uniform(0.75, 1.0),
        )
        bin_config["color"] = [*rgb, 1.0]
    worldbody.append(
        _classification_bin_body(bin_config, center_xy, table_surface_z)
    )
    print(
        f"[INFO] Added food_bin at [{center_xy[0]:.3f}, "
        f"{center_xy[1]:.3f}, {table_surface_z:.3f}]"
    )
    return {
        "name": str(bin_config["name"]),
        "center_xy": center_xy,
        "color": list(bin_config["color"]),
    }


def _euler_to_rotation_matrix(euler):
    """Convert XYZ Euler angles (radians) to a 3x3 rotation matrix.

    Matches MuJoCo's default euler sequence (XYZ): R = Rz(rz) @ Ry(ry) @ Rx(rx).
    """
    rx, ry, rz = euler
    cx, sx = np.cos(rx), np.sin(rx)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    cy, sy = np.cos(ry), np.sin(ry)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    cz, sz = np.cos(rz), np.sin(rz)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _compute_mesh_placement_info(assets, orientation=None):
    """Compute mesh placement info accounting for the YCB coordinate frame.

    Each YCB object model defines its own coordinate frame which is usually NOT
    at the object center and NOT aligned with principal geometric axes.  This
    function reads the visual and collision OBJ meshes, applies the desired
    orientation, then returns:
      - collision_z_min: minimum Z across every rotated collision mesh
      - centroid_x / centroid_y: visual-mesh centre in rotated XY

    The caller uses these to centre the object on a grid position and to place
    the collision bottom just above the table regardless of the coordinate
    frame.  Visual XY remains unchanged because that defines the operator-facing
    scene layout.
    """
    if orientation is None:
        orientation = [0.0, 0.0, 0.0]

    visual_verts = load_obj_vertices(assets.visual_mesh)
    collision_verts = np.concatenate(
        [load_obj_vertices(path) for path in assets.collision_meshes],
        axis=0,
    )

    if np.any(np.array(orientation) != 0):
        R = _euler_to_rotation_matrix(orientation)
        visual_verts = (R @ visual_verts.T).T
        collision_verts = (R @ collision_verts.T).T

    return {
        'collision_z_min': float(collision_verts[:, 2].min()),
        'centroid_x': float(visual_verts[:, 0].mean()),
        'centroid_y': float(visual_verts[:, 1].mean()),
    }


def build_ycb_object_xml(object_name, assets):
    """Build one MuJoCo object from validated numbered-YCB assets."""
    root = ET.Element("mujoco", model=object_name)
    asset = ET.SubElement(root, "asset")
    body = ET.SubElement(
        ET.SubElement(root, "worldbody"),
        "body",
        name=object_name,
    )
    ET.SubElement(body, "joint", name=f"{object_name}_joint", type="free")

    texture_name = f"{object_name}_tex"
    material_name = f"{object_name}_mat"
    visual_mesh_name = f"{object_name}_visual_mesh"
    ET.SubElement(
        asset,
        "texture",
        name=texture_name,
        type="2d",
        file=str(assets.texture),
    )
    ET.SubElement(
        asset,
        "material",
        name=material_name,
        texture=texture_name,
    )
    ET.SubElement(
        asset,
        "mesh",
        name=visual_mesh_name,
        file=str(assets.visual_mesh),
    )
    ET.SubElement(
        body,
        "geom",
        name=f"{object_name}_visual",
        type="mesh",
        mesh=visual_mesh_name,
        material=material_name,
        contype="0",
        conaffinity="0",
        group="2",
    )

    for index, collision_path in enumerate(assets.collision_meshes):
        mesh_name = f"{object_name}_collision_mesh_{index}"
        ET.SubElement(
            asset,
            "mesh",
            name=mesh_name,
            file=str(collision_path),
        )
        geom_attributes = {
            "name": f"{object_name}_collision_{index}",
            "type": "mesh",
            "mesh": mesh_name,
            "rgba": "0.8 0.8 0.8 0.25",
            "friction": "0.9 0.2 0.05",
            "group": "3",
        }
        if index == 0:
            geom_attributes["mass"] = "0.1"
        ET.SubElement(body, "geom", **geom_attributes)

    return root


def _layout_grid(x_range, y_range, n, margin=0.05):
    """Generate n positions in a grid pattern within the given x/y ranges."""
    cols = max(1, int(math.ceil(math.sqrt(n * (x_range[1] - x_range[0]) /
                                            (y_range[1] - y_range[0])))))
    rows = max(1, int(math.ceil(n / cols)))

    x_step = (x_range[1] - x_range[0] - 2 * margin) / max(cols, 1)
    y_step = (y_range[1] - y_range[0] - 2 * margin) / max(rows, 1)
    x0 = x_range[0] + margin + x_step / 2
    y0 = y_range[0] + margin + y_step / 2

    positions = []
    for i in range(n):
        col = i % cols
        row = i // cols
        jitter_x = random.uniform(-x_step * 0.3, x_step * 0.3)
        jitter_y = random.uniform(-y_step * 0.3, y_step * 0.3)
        positions.append([x0 + col * x_step + jitter_x,
                          y0 + row * y_step + jitter_y])
    random.shuffle(positions)
    return positions


def select_scene_objects(objects_config, random_object_count=0, fixed_object_names=None, rng=random):
    """Select scene objects while always keeping configured fixed objects."""
    if not objects_config:
        return []

    fixed_object_names = list(fixed_object_names or [])
    by_name = {obj['name']: obj for obj in objects_config}

    missing = [name for name in fixed_object_names if name not in by_name]
    if missing:
        raise ValueError(
            "fixed_object_names contains unknown object(s): "
            + ", ".join(missing)
        )

    selected = []
    selected_names = set()
    for name in fixed_object_names:
        if name in selected_names:
            continue
        selected.append(by_name[name])
        selected_names.add(name)

    if random_object_count <= 0:
        if selected:
            return selected
        return list(objects_config)

    target_count = max(random_object_count, len(selected))
    remaining = [
        obj for obj in objects_config
        if obj['name'] not in selected_names
    ]
    fill_count = min(max(0, target_count - len(selected)), len(remaining))
    if fill_count:
        selected.extend(rng.sample(remaining, fill_count))

    return selected


def normalize_scene_mode(scene_mode):
    """Normalize and validate a simulator scene mode."""
    normalized = str(scene_mode).strip().lower()
    if normalized not in SCENE_MODES:
        raise ValueError(
            "scene_mode must be one of: mix, random, assign; "
            f"got {scene_mode!r}"
        )
    return normalized


def select_category_scene_objects(
    objects_config,
    scene_object_categories,
    rng=random,
):
    """Select one object per ordered category plus one remaining-pool object."""
    objects_config = list(objects_config or [])
    if not objects_config:
        raise ValueError("random scene selection requires a non-empty object pool")

    if scene_object_categories is None or not hasattr(
        scene_object_categories,
        "keys",
    ):
        raise ValueError("scene_object_categories must be a category mapping")

    configured_categories = list(scene_object_categories.keys())
    missing_categories = [
        category
        for category in RANDOM_SCENE_CATEGORY_ORDER
        if category not in configured_categories
    ]
    if missing_categories:
        raise ValueError(
            "scene_object_categories is missing required category/categories: "
            + ", ".join(missing_categories)
        )

    unexpected_categories = [
        category
        for category in configured_categories
        if category not in RANDOM_SCENE_CATEGORY_ORDER
    ]
    if unexpected_categories:
        raise ValueError(
            "scene_object_categories contains unexpected category/categories: "
            + ", ".join(unexpected_categories)
        )

    object_names = [obj["name"] for obj in objects_config]
    duplicate_pool_names = sorted(
        name for name in set(object_names) if object_names.count(name) > 1
    )
    if duplicate_pool_names:
        raise ValueError(
            "object pool contains duplicate name(s): "
            + ", ".join(duplicate_pool_names)
        )
    by_name = {obj["name"]: obj for obj in objects_config}

    validated_categories = {}
    category_by_object = {}
    for category in RANDOM_SCENE_CATEGORY_ORDER:
        raw_names = scene_object_categories[category]
        category_names = list(raw_names) if raw_names is not None else []
        if not category_names:
            raise ValueError(
                f"scene category {category!r} must not be empty"
            )

        for name in category_names:
            if name not in by_name:
                raise ValueError(
                    f"scene category {category!r} contains {name!r}, "
                    "which is not in the object pool"
                )
            previous_category = category_by_object.get(name)
            if previous_category is not None:
                raise ValueError(
                    f"object {name!r} appears in both scene categories "
                    f"{previous_category!r} and {category!r}"
                )
            category_by_object[name] = category

        validated_categories[category] = category_names

    selected_names = [
        rng.choice(validated_categories[category])
        for category in RANDOM_SCENE_CATEGORY_ORDER
    ]
    selected_name_set = set(selected_names)
    remaining_objects = [
        obj for obj in objects_config if obj["name"] not in selected_name_set
    ]
    if not remaining_objects:
        raise ValueError(
            "random scene requires a sixth object, but no remaining object "
            "is available after category selection"
        )

    selected = [by_name[name] for name in selected_names]
    selected.append(rng.choice(remaining_objects))
    return selected


def select_assigned_scene_objects(
    objects_config,
    assigned_names=None,
    rng=random,
):
    """Keep assigned objects in slot order and randomly fill to six."""
    objects_config = list(objects_config or [])
    assigned_names = list(assigned_names or [])

    if len(assigned_names) > SCENE_OBJECT_COUNT:
        raise ValueError(
            f"assign mode accepts at most {SCENE_OBJECT_COUNT} objects; "
            f"got {len(assigned_names)}"
        )

    by_normalized_name = {}
    for obj in objects_config:
        name = obj.get("name") if hasattr(obj, "get") else None
        if not isinstance(name, str) or not name.strip():
            raise ValueError("object pool entries must have a valid name")
        normalized_name = name.strip().casefold()
        if normalized_name in by_normalized_name:
            raise ValueError(
                f"object pool contains duplicate name: {normalized_name}"
            )
        by_normalized_name[normalized_name] = obj

    if len(by_normalized_name) < SCENE_OBJECT_COUNT:
        raise ValueError(
            "assign mode requires at least "
            f"{SCENE_OBJECT_COUNT} unique objects in the pool"
        )

    selected = []
    selected_normalized_names = set()
    for raw_name in assigned_names:
        normalized_name = str(raw_name).strip().casefold()
        if normalized_name not in by_normalized_name:
            raise ValueError(f"unknown assigned object: {raw_name}")
        if normalized_name in selected_normalized_names:
            raise ValueError(
                f"duplicate assigned object: "
                f"{by_normalized_name[normalized_name]['name']}"
            )
        selected.append(by_normalized_name[normalized_name])
        selected_normalized_names.add(normalized_name)

    remaining = [
        obj
        for normalized_name, obj in by_normalized_name.items()
        if normalized_name not in selected_normalized_names
    ]
    fill_count = SCENE_OBJECT_COUNT - len(selected)
    selected.extend(rng.sample(remaining, fill_count))
    return selected


def select_scene_objects_for_mode(
    objects_config,
    scene_mode="mix",
    random_object_count=0,
    fixed_object_names=None,
    scene_object_categories=None,
    assigned_object_names=None,
    rng=random,
):
    """Dispatch scene selection while retaining the existing mix selector."""
    normalized_mode = normalize_scene_mode(scene_mode)
    if normalized_mode == "mix":
        return select_scene_objects(
            objects_config,
            random_object_count=random_object_count,
            fixed_object_names=fixed_object_names,
            rng=rng,
        )

    if normalized_mode == "random":
        return select_category_scene_objects(
            objects_config,
            scene_object_categories,
            rng=rng,
        )

    return select_assigned_scene_objects(
        objects_config,
        assigned_names=assigned_object_names,
        rng=rng,
    )


def assign_placement_slots(objects_config, placement_slots):
    """Return object configs copied onto fixed placement slots."""
    placement_slots = list(placement_slots or [])
    if not placement_slots:
        return list(objects_config or [])

    objects_config = list(objects_config or [])
    if len(placement_slots) < len(objects_config):
        raise ValueError(
            f"placement_slots has {len(placement_slots)} entries but "
            f"{len(objects_config)} objects were selected"
        )

    assigned = []
    for obj_config, slot in zip(objects_config, placement_slots):
        if len(slot) != 3:
            raise ValueError(
                f"placement_slots entries must be [x, y, z], got {slot!r}"
            )
        copied = dict(obj_config)
        copied.pop("position_range", None)
        copied["position"] = [float(slot[0]), float(slot[1]), float(slot[2])]
        assigned.append(copied)
    return assigned


def populate_scene(model_path, objects_config=None, camera_names=[],
                   random_object_count=0, fixed_object_names=None,
                   table_surface_z=-0.025, classification_bins=None):
    """ Populate the MuJoCo scene with objects defined in the configuration.

    Parse the XML model, add objects, and return the modified XML as a string.

    Args:
        model_path: Path to the base MuJoCo XML model
        objects_config: List of object configurations with name, type, size, position, etc.
        camera_names: List of camera names to configure in the scene
        random_object_count: If >0, randomly select this many objects from objects_config
        fixed_object_names: Names that should always appear before random fill objects
        table_surface_z: Z coordinate of the table surface in world frame (default -0.025)

    Returns:
        Modified XML as a string
    """
    tree = ET.parse(model_path)
    root = tree.getroot()
    base_dir = os.path.dirname(os.path.abspath(model_path))

    resolve_includes(root, base_dir)
    resolve_all_file_paths(root, base_dir)
    configure_cameras_in_scene(root, camera_names)

    # Remove existing objects that might conflict
    worldbody = root.find('worldbody')
    if worldbody is None:
        raise ValueError("No worldbody found in XML model")

    validate_sorting_layout(objects_config, classification_bins)
    add_classification_bins(
        worldbody,
        classification_bins,
        table_surface_z=table_surface_z,
    )

    # Add objects to the scene if provided
    if objects_config:
        selected = select_scene_objects(
            objects_config,
            random_object_count=random_object_count,
            fixed_object_names=fixed_object_names,
        )
        if fixed_object_names:
            selected_names = ", ".join(obj['name'] for obj in selected)
            print(f"[INFO] Selected fixed-priority scene objects: {selected_names}")
        elif random_object_count > 0 and len(objects_config) > random_object_count:
            print(f"[INFO] Randomly selected {random_object_count}/{len(objects_config)} objects")

        # Generate grid layout positions for x/y
        if selected and 'position_range' in selected[0]:
            pr = selected[0]['position_range']
            xy_positions = _layout_grid([pr['x'][0], pr['x'][1]],
                                        [pr['y'][0], pr['y'][1]],
                                        len(selected))
        else:
            xy_positions = [None] * len(selected)

        for i, obj_config in enumerate(selected):
            if 'position_range' in obj_config:
                pr = obj_config['position_range']
                xy = xy_positions[i]
                z_sample = random.uniform(*pr['z'])
            else:
                pos = obj_config['position']
                xy = [pos[0], pos[1]]
                z_sample = pos[2]

            if obj_config.get('type') == 'mesh':
                # Each YCB model has its own coordinate frame, usually not at
                # the object centre.  Compute the geometric centre (XY) and the
                # bottom (Z) of the mesh *after* applying the desired orientation
                # so the object is correctly centred and sits on the table.
                ori = obj_config.get('orientation', [0, 0, 0])
                assets = resolve_ycb_assets(
                    obj_config['name'],
                    obj_config.get('ycb_dir'),
                )
                placement = _compute_mesh_placement_info(
                    assets,
                    orientation=ori,
                )

                body_x = xy[0] - placement['centroid_x']
                body_y = xy[1] - placement['centroid_y']
                body_z = (
                    table_surface_z
                    - placement['collision_z_min']
                    + MESH_SPAWN_CLEARANCE_M
                    + z_sample
                )
                pos = [body_x, body_y, body_z]

                ycb_root = build_ycb_object_xml(obj_config['name'], assets)

                # Merge <asset> elements into scene
                scene_asset = root.find('asset')
                if scene_asset is None:
                    scene_asset = ET.SubElement(root, 'asset')
                ycb_asset = ycb_root.find('asset')
                if ycb_asset is not None:
                    for child in list(ycb_asset):
                        scene_asset.append(child)

                # Extract body, set position/orientation
                ycb_body = ycb_root.find('worldbody/body')
                if ycb_body is None:
                    raise ValueError(
                        f"Generated YCB body missing for object "
                        f"{obj_config['name']!r} in {assets.directory}"
                    )
                ycb_body.set('pos', f"{pos[0]} {pos[1]} {pos[2]}")
                ycb_body.set('euler', f"{ori[0]} {ori[1]} {ori[2]}")
                worldbody.append(ycb_body)
            else:
                pos = [xy[0], xy[1], z_sample]
                obj_config = dict(obj_config)
                obj_config['position'] = pos
                obj_body = create_object_xml(obj_config)
                worldbody.append(obj_body)
            if obj_config.get('type') == 'mesh':
                print(f"[INFO] Added {obj_config['name']} (mesh) to scene at [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] "
                      f"(collision_z_min={placement['collision_z_min']:.3f}, "
                      f"centroid_xy=[{placement['centroid_x']:.3f}, {placement['centroid_y']:.3f}])")
            else:
                print(f"[INFO] Added {obj_config['name']} ({obj_config['type']}) to scene at [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}]")

    return ET.tostring(root, encoding='unicode')
