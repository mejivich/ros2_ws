from setuptools import setup
import os
from glob import glob

package_name = 'ur_yt_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    data_files=[
        # Resource index
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name] if os.path.exists("resource/" + package_name) else []),

        # Package.xml
        ("share/" + package_name, ["package.xml"]),

        # Launch files
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),

        # URDF/XACRO
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf") + glob("urdf/*.xacro")),

        # Configs
        ("share/" + package_name + "/config", glob("config/*.yaml")),

        # RViz
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),

        # Worlds
        ("share/" + package_name + "/worlds", glob("worlds/*.world")),

        # Meshes (recursive)
        ("share/" + package_name + "/meshes", glob("meshes/**/*", recursive=True)),

        # Gazebo models (recursive)
        ("share/" + package_name + "/models", glob("models/**/*", recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='UR16e + car + MoveIt + Gazebo integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)