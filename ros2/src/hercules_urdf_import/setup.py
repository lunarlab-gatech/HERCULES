from glob import glob

from setuptools import find_packages, setup

package_name = 'hercules_urdf_import'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John C. Furey',
    maintainer_email='john.c.furey@erdc.dren.mil',
    description='Import URDF robots into HERCULES (settings, Unreal visuals, ROS 2 TF).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hercules_urdf_import = hercules_urdf_import.cli:main',
        ],
    },
)
