from setuptools import setup
import os
from glob import glob

def recursive_files(path):
    """Return a list of all files under path (ignore directories)."""
    files = []
    for root, _, filenames in os.walk(path):
        for f in filenames:
            files.append(os.path.join(root, f))
    return files

package_name = 'ur_yt_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['package.xml']),
        ('share/' + package_name, ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),

        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # URDF/XACRO files
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf') + glob('urdf/*.xacro')),

        # Config files
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/ur16e', glob('config/ur16e/*.yaml')),

        # Meshes, models, rviz, worlds — only files, recursive
        ('share/' + package_name + '/meshes', recursive_files('meshes')),
        ('share/' + package_name + '/models', recursive_files('models')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
)