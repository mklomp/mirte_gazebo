#!/usr/bin/env python


import os, sys
import yaml


def get_file_path(sub_dir, file_name, default_ext):
    """
    Get the absolute path of a resource file, which may be relative to
    the gym_duckietown module directory, or an absolute path.

    This function is necessary because the simulator may be imported by
    other packages, and we need to be able to load resources no matter
    what the current working directory is.
    """

    assert "." not in default_ext
    assert "/" not in default_ext

    # If this is already a real path
    if os.path.exists(file_name):
        return file_name

    subdir_path = get_subdir_path(sub_dir)
    file_path = os.path.join(subdir_path, file_name)

    if "." not in file_name:
        file_path += "." + default_ext

    return file_path


def loadMap(map_name):
    """
    Load the map layout from a YAML file
    """

    # Store the map name

    # Get the full map file path
    map_file_path = get_file_path("maps", map_name, "yaml")

    # print('loading map file "%s"' % map_file_path)

    with open(map_file_path, "r") as f:
        map_data = yaml.load(f, Loader=yaml.Loader)

    if not "tile_size" in map_data:
        msg = "Must now include explicit tile_size in the map data."
        raise ValueError(msg)
    road_tile_size = map_data["tile_size"]
    # self._init_vlists()

    tiles = map_data["tiles"]
    assert len(tiles) > 0
    assert len(tiles[0]) > 0

    # Create the grid
    grid_height = len(tiles)
    grid_width = len(tiles[0])
    grid = [None] * grid_width * grid_height

    # We keep a separate list of drivable tiles
    drivable_tiles = []

    # For each row in the grid
    for j, row in enumerate(tiles):
        msg = "each row of tiles must have the same length"
        if len(row) != grid_width:
            raise Exception(msg)

        # For each tile in this row
        for i, tile in enumerate(row):
            tile = tile.strip()

            if tile == "empty":
                continue

            if "/" in tile:
                it = tile.split("/")
                kind = it[0]
                orient = it[1]

                kind = kind.strip(" ")
                orient = orient.strip(" ")
                # if(orient == ):
                #     angle = 0
                angle = ["N", "E", "S", "W"].index(orient)
                drivable = True
            elif "4" in tile:
                kind = "4way"
                angle = 2
                drivable = True
            else:
                kind = tile
                angle = 0
                drivable = False
            if(kind.startswith('R')):
                kind = "RCJ/"+kind[1:]
            else:
                kind = "DT/" + kind
            tile = {
                "coords": (i, j),
                "kind": kind,
                "angle": angle,
                "drivable": drivable,
            }

            set_tile(i, j, tile)

            if drivable:
                pass
                # tile['curves'] = self._get_curve(i, j)
                # self.drivable_tiles.append(tile)

    # self.mesh = ObjMesh.get('duckiebot')
    _load_objects(map_data)

    # Get the starting tile from the map, if specified
    # self.start_tile = None
    # if 'start_tile' in map_data:
    #     coords = map_data['start_tile']
    #     self.start_tile = self._get_tile(*coords)


def _load_objects(map_data):
    # Create the objects array
    objects = []
    objects = map_data.get('objects', [])
    if(objects is None):
        return
    # For each object
    for obj_idx, desc in enumerate(map_data.get("objects", [])):
        kind = desc["kind"]

        pos = desc["pos"]
        x, z = pos[0:2]
        x = eval(str(x))
        z = eval(str(z))
        y = pos[2] if len(pos) == 3 else 0.0
        y = eval(str(y))
        rotate = desc["rotate"]
        optional = desc.get("optional", False)

        # pos = road_tile_size * np.array((x, y, z))

        # Load the mesh
        # mesh = ObjMesh.get(kind)
        if "static" in desc:
            static = desc["static"]
        else:
            static = True
        if "height" in desc:
            scale = desc["height"]
        else:
            scale = desc["scale"]
        if "included" in kind:
            set_object_included(x, z, kind.replace("included", ""), -rotate, static, scale)
        else:
            set_object(x, z, kind, -rotate, static, scale)


# <xacro:tile name="straight_1" material="DT/Straight" size="0.6" x="0" y="0"/>
#   <xacro:tile name="curve_1" material="DT/Curve" size="0.6" x="1" y="0" yaw="90"/>
#   <xacro:tile name="curve_2" material="DT/Curve" size="0.6" x="1" y="-1"/>
#   <xacro:tile name="straight_2" material="DT/Straight" size="0.6" x="0" y="-1"/>
# tile = {
#                     'coords': (i, j),
#                     'kind': kind,
#                     'angle': angle,
#                     'drivable': drivable
#                 }

tiles_world = (
    []
)  # <xacro:tileWorld name="tile_8_8" material="DT/asphalt" size="0.6" x="8" y="8" yaw="0"/>
tiles_state = (
    []
)  # <xacro:tileState name="tile_1_0" material="DT/asphalt" size="0.6" x="1" y="0" yaw="0"/>

# <xacro:duckieModel name="name_sign_blank" type="sign_blank" size="0.6" x="0" y="0" yaw="0"/>
#       <xacro:duckieState name="name_sign_blank" type="sign_blank" size="0.6" x="2" y="2"  yaw="0"/>
existing_objects = []

def set_object(i, j, type, angle, static, scale):
    # return
    # print(i, j, type, angle, static, scale)
    # scale = 1
    i = i - 0.5
    j = j - 0.5
    i_ = str(i).replace(".", "_")
    j_ = str(j).replace(".", "_")
    name = f'obj_{i_}_{j_}'
    if(name in existing_objects):
        return
    existing_objects.append(name)
    tiles_world.append(
        f'<xacro:duckieModel name="obj_{i_}_{j_}" type="{type}" size="{size}" x="{i}" y="{j}" yaw="{angle}" static="${{{static}}}" scale="${{{scale}}}"/>'
    )
    tiles_state.append(
        f'<xacro:duckieState name="obj_{i_}_{j_}" type="{type}" size="{size}" x="{i}" y="{j}" yaw="{angle}" static="${{{static}}}" scale="${{{scale}}}"/>'
    )

def set_object_included(i, j, type, angle, static, scale):
    # print(i, j, type, angle, static, scale)
    # scale = 1
    i = i - 0.5
    j = j - 0.5
    i_ = str(i).replace(".", "_")
    j_ = str(j).replace(".", "_")
    tiles_world.append(
        f'<xacro:includedModel name="obj_{i_}_{j_}" type="{type}" size="{size}" x="{i}" y="{j}" yaw="{angle}" static="${{{static}}}" scale="${{{scale}}}"/>'
    )
    tiles_state.append(
        f'<xacro:includedState name="obj_{i_}_{j_}" type="{type}" size="{size}" x="{i}" y="{j}" yaw="{angle}" static="${{{static}}}" scale="${{{scale}}}"/>'
    )

size = 0.5

def set_tile(i, j, tile):
    tiles_world.append(
        f"<xacro:tileWorld name=\"tile_{i}_{j}\" material=\"{tile['kind']}\" size=\"{size}\" x=\"{i}\" y=\"{j}\" yaw=\"{tile['angle']*90}\"/>"
    )
    tiles_state.append(
        f"<xacro:tileState name=\"tile_{i}_{j}\" material=\"{tile['kind']}\" size=\"{size}\" x=\"{i}\" y=\"{j}\" yaw=\"{tile['angle']*90}\"/>"
    )


mapN = sys.argv[1]
dest = sys.argv[2]
loadMap(mapN)

# grid = "\n".join(tiles)

empty_world_file = os.path.join(
    os.path.dirname(__file__), "../urdf/empty_duckietown.world"
)

with open(empty_world_file, "r") as file:
    data = file.read()
    data = data.replace("<!-- TILEWORLD -->", "\n".join(tiles_world)).replace(
        "<!-- TILESTATE -->", "\n".join(tiles_state)
    )
    with open(dest, "w") as f:
        f.write(data)
