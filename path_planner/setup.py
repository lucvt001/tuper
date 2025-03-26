from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='smarc2user',
    maintainer_email='luc001@e.ntu.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_lead_path = path_planner.generate_lead_path:main',
            'generate_offset_path = path_planner.generate_offset_path:main',
            'visualize_paths = path_planner.visualize_paths:main',
            'csv_tf_broadcaster = path_planner.csv_tf_broadcaster:main',
        ],
    },
)
