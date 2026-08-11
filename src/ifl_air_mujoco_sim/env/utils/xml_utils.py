import os
import copy
import xml.etree.ElementTree as ET
import numpy as np

# Helper: recursively resolve simple MuJoCo-style <include file="..."/>
def resolve_includes(elem, base_dir):
    # Work on a copy of children list because we'll mutate
    for child in list(elem):
        tag = child.tag if isinstance(child.tag, str) else ''
        # Accept plain 'include' or namespaced variants that end with 'include'
        if tag.lower().endswith('include'):
            href = child.get('file') or child.get('href') or child.get('src')
            if not href:
                # nothing to include; remove the include element
                elem.remove(child)
                continue
            inc_path = href if os.path.isabs(href) else os.path.join(base_dir, href)
            try:
                inc_tree = ET.parse(inc_path)
                inc_root = inc_tree.getroot()
                # Recursively resolve includes in the included file
                resolve_includes(inc_root, os.path.dirname(inc_path))
                # Replace the <include/> element with the children of the included root
                idx = list(elem).index(child)
                elem.remove(child)
                for k, inc_child in enumerate(list(inc_root)):
                    elem.insert(idx + k, copy.deepcopy(inc_child))
            except Exception as e:
                # If include fails, log and remove the include element to avoid loops
                try:
                    print('Failed to include "%s": %s', inc_path, str(e))
                except Exception:
                    pass
                elem.remove(child)
        else:
            # Recurse into non-include children
            resolve_includes(child, base_dir)

def resolve_asset_paths(root_elem, base_dir):
    """Resolve all relative file paths in the assets section to absolute paths.
    
    Args:
        root_elem: XML root element
        base_dir: Base directory for resolving relative paths
    """
    # Find the asset section
    asset_elem = root_elem.find('asset')
    if asset_elem is None:
        return
    
    # Asset elements that can have file attributes
    asset_types_with_files = [
        'texture',  # file attribute
        'material', # texture references are handled separately
        'mesh',     # file attribute
        'skin',     # file attribute
        'hfield',   # file attribute
    ]
    
    for asset_type in asset_types_with_files:
        for elem in asset_elem.findall(asset_type):
            # Handle 'file' attribute
            file_attr = elem.get('file')
            if file_attr and not os.path.isabs(file_attr):
                abs_path = os.path.join(base_dir, "assets", file_attr)
                elem.set('file', abs_path)
                
            # Handle texture-specific attributes
            if asset_type == 'texture':
                # Some textures might use 'content' or other attributes
                for attr in ['content']:
                    attr_value = elem.get(attr)
                    if attr_value and not os.path.isabs(attr_value):
                        abs_path = os.path.join(base_dir, "assets", attr_value)
                        elem.set(attr, abs_path)
    
    # Also handle compiler meshdir attribute if present
    compiler_elem = root_elem.find('compiler')
    if compiler_elem is not None:
        meshdir = compiler_elem.get('meshdir')
        if meshdir and not os.path.isabs(meshdir):
            abs_meshdir = os.path.join(base_dir, meshdir)
            compiler_elem.set('meshdir', abs_meshdir)

def resolve_all_file_paths(root_elem, base_dir):
    """Resolve all file paths in the XML (includes, assets, and other file references).
    
    Args:
        root_elem: XML root element
        base_dir: Base directory for resolving relative paths
    """
    # First resolve includes
    resolve_includes(root_elem, base_dir)
    
    # Then resolve asset paths
    resolve_asset_paths(root_elem, base_dir)
    
    # Handle other potential file references
    # Add more as needed for your specific use case
    file_reference_elements = [
        ('.//body[@file]', 'file'),
        ('.//geom[@file]', 'file'),
        ('.//site[@file]', 'file'),
    ]
    
    for xpath, attr in file_reference_elements:
        for elem in root_elem.findall(xpath):
            file_path = elem.get(attr)
            if file_path and not os.path.isabs(file_path):
                abs_path = os.path.join(base_dir, file_path)
                elem.set(attr, abs_path)


def extract_actuator_names(model_path: str):
    """Extract actuator names from a MuJoCo XML model, resolving includes."""
    actuators = []  # list of (actuator_name, joint_name)
    if model_path is not None:
        try:
            tree = ET.parse(model_path)
            root = tree.getroot()
            resolve_includes(root, os.path.dirname(model_path) or '.')

            for act in root.findall('.//actuator'):
                for child in act:
                    act_name = child.get('name')
                    if act_name:
                        actuators.append(act_name)
        except Exception:
            actuators = []
    return actuators