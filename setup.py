from setuptools import setup
from glob import glob

package_name = 'mirte_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch' , glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name , ['test_world.world']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mirte',
    maintainer_email='m.klomp@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
)
