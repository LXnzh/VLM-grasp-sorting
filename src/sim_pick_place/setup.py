import os
from setuptools import find_packages, setup

package_name = 'sim_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install icon
        (os.path.join('share', package_name, 'icons'), ['icons/sim_reset_icon.png']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edgar',
    maintainer_email='edgar.welte@kit.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_pick_place_node = sim_pick_place.sim_pick_place_node:main',
            'sim_stack_cubes_node = sim_pick_place.sim_stack_cubes_node:main',
            'sim_reset_gui_node = sim_pick_place.sim_reset_gui_node:main',
        ],
    },
)
