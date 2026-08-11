        # model path is only robot, gripper and table
        # add objects to scene dynamically
        #tree = ET.parse(model_path)
        #root = tree.getroot()
        #worldbody = root.find('worldbody')
#
        #cube_init_pos = '0.0 0.4 0.1'  # Initial position of the cube
        #cube_init_quat = '1 0 0 0'
#
        #if worldbody is None:
        #    raise ValueError("No 'worldbody' element found in the model XML.")
        #red_cube = ET.SubElement(worldbody, 'body', name='red_cube', pos=cube_init_pos, quat=cube_init_quat)
        #ET.SubElement(red_cube, 'joint', type='free')
        #ET.SubElement(red_cube, 'inertial', pos='0 0 0', mass='1.0', diaginertia='0.0001 0.0001 0.0001')
        #ET.SubElement(red_cube, 'geom', type='box', size='0.02 0.02 0.02', rgba='1 0 0 1')
#
        #keyframe = root.find('keyframe')
        #if keyframe is not None:
        #    for key in keyframe.findall('key'):
        #        qpos = key.get('qpos')
        #        if qpos is not None:
        #            cube_pose = cube_init_pos + ' ' + cube_init_quat
        #            cube_pose = np.array(cube_pose.strip().split(), dtype=float)


        mjcf_model = mjcf.from_path(model_path)

        assets_copy = [copy.deepcopy(asset) for asset in mjcf_model.asset.all_children()]

        # add cube
        # don't add name because dm_control will rehash all existing references to assets
        red_cube = mjcf_model.worldbody.add('body', pos=[0.0, 0.4, 0.1], quat=[1, 0, 0, 0])
        red_cube.add(
            'geom',
            type='box',
            size=[0.02, 0.02, 0.02],
            rgba=[1, 0, 0, 1]
        )

        # Restore file-based assets
        mjcf_model.asset.clear()
        for asset in assets_copy:
            mjcf_model.asset.append(asset)

        # fix meshdir='assets' deleted by dm_control
        #model_xml_str = mjcf_model.to_xml_string()
        #root_xml_element = mjcf_model._root
        #compiler = root_xml_element.find('compiler')
        #if compiler is None:
        #    compiler = ET.SubElement(root_xml_element, 'compiler')
        #compiler.set('meshdir', 'assets')
        
        model_xml_str = ET.tostring(root_xml_element, encoding='unicode')

        # save changed xml in temporary file at same path as model_path
        with tempfile.NamedTemporaryFile(suffix='.xml', delete=False, dir=os.path.dirname(model_path)) as self.tmp_xml_file:
            self.tmp_xml_file.write(model_xml_str.encode('utf-8'))
            print(f"[INFO]: Temporary model file created at {model_path}")