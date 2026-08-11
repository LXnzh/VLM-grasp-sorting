import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arm_api2_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', 'examples'), glob('examples/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edgar',
    maintainer_email='edgar.welte@kit.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "arm_api2_client_example = examples.arm_api2_client_example:main",
            "reset_pose = examples.reset_pose:main",
        ],
    },
)
