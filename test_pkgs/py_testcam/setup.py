from setuptools import find_packages, setup

package_name = 'py_testcam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arendjan',
    maintainer_email='jemoeder@arend-jan.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = py_testcam.listener:main',
            'relistener = py_testcam.listener_sender:main',
            'pc = py_testcam.listener_pc:main',
                        'prelistener = py_testcam.listener_sender_points:main'

        ],
    },
)
