from setuptools import setup
from glob import glob

package_name = 'mirte_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name, ['launch/gazebo.xml']),
        ('share/' + package_name + '/urdf', ['urdf/materials.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/macros.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/ros2_control.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/mirte.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/lidar.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/camera.xacro']),
        ('share/' + package_name + '/config', ['config/mirte_dimensions.yaml']),
        ('share/' + package_name + '/config', ['config/mirte_diff_drive.yaml']),
        ('share/' + package_name + '/meshes', ['meshes/mirte_base.stl']),
        ('share/' + package_name , ['test_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mirte',
    maintainer_email='m.klomp@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
)
