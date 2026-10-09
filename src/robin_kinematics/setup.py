from glob import glob

from setuptools import find_packages, setup

package_name = 'robin_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bima',
    maintainer_email='bimaaditya2343@gmail.com',
    description='Forward dan inverse kinematics differential drive robot robin',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'inverse_kinematics_node = '
            'robin_kinematics.inverse_kinematics_node:main',
            'forward_kinematics_node = '
            'robin_kinematics.forward_kinematics_node:main',
        ],
    },
)
