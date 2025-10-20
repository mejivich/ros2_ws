from setuptools import setup

package_name = 'ur_yt_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'launch/ur16e_with_moveit.launch.py',
            'urdf/ur16e_camera.urdf.xacro',
            'urdf/car_with_inlet.urdf',
            'worlds/world2.world',
            'config/ur16e_controllers.yaml',
            'config/initial_positions.yaml',
            'rviz/view_ur16e.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='UR16e and car simulation package',
    license='TODO',
    tests_require=['pytest'],
)