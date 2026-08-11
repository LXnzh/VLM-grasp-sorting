from setuptools import find_packages, setup

package_name = 'my_course_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'openai',
        'pycocotools',
        'requests',
    ],
    zip_safe=True,
    maintainer='pmlrs04',
    maintainer_email='wenyuanzzhang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'verify_init_pose = my_course_pkg.verify_init_pose:main',
            'rgbd_saver = my_course_pkg.rgbd_file_save:main',
            'pipeline = my_course_pkg.pipeline:main',
            'grasp_demo = my_course_pkg.grasp.demo:run',
            'grasp_plan_only = my_course_pkg.grasp.plan_only:run',
            'grasp_eval = my_course_pkg.grasp_eval:main',
            'experiment_session = my_course_pkg.experiment_session:main',
            'sim_target_nudge = '
            'my_course_pkg.tasks.tracking.sim_target_nudge:main',
            'foundationpose_tracking_grasp = '
            'my_course_pkg.tasks.tracking.node:main',
            'pbvs_sorting_grasp = my_course_pkg.tasks.pbvs.node:main',
        ],
    },
)
