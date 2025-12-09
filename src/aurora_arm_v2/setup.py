from setuptools import find_packages, setup
from glob import glob

package_name = 'aurora_arm_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (('share/' + package_name + '/description'), glob('description/*.urdf')),
        (('share/' + package_name + '/meshes'), glob('meshes/*.stl')),
        # (('share/' + package_name + '/config'), glob('config/*.yaml')),
        # (('share/' + package_name + '/world'), glob('world/*.world')),
        # (('share/' + package_name + '/launch'), glob('launch/*.launch.py')),
        (('share/' + package_name + '/description'),glob('description/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fuga1129',
    maintainer_email='sakihama1129@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
