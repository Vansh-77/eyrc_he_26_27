from setuptools import find_packages, setup
import os

package_name = 'prep'

data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    )
]

for root, dirs, files in os.walk('models'):
    if files:
        install_dir = os.path.join(
            'share',
            package_name,
            root
        )

        data_files.append(
            (
                install_dir,
                [os.path.join(root, f) for f in files]
            )
        )

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vansh',
    maintainer_email='vansh@todo.todo',
    description='eYRC robotics preparation package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'mujoco_sim = prep.mujoco_sim:main',
             'camera_node = prep.camera_node:main',
             'localization_node = prep.localization_node:main',
             'control_node = prep.control:main'
        ],
    },
)