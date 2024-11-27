import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'urdf_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  
        (os.path.join('share', package_name, 'world'), glob('world/*.sdf')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dy',
    maintainer_email='wjm2517@hanyang.ac.kr',
    description='urdf package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #fake_state_publisher 치면 urdfpkg에 fake_state_publihser파일 속 main file 실행
            'fake_state_publisher = urdf_pkg.fake_state_publisher:main',
            'state_publisher = urdf_pkg.state_publisher:main',
        ],
    },
)
