from setuptools import setup
from glob import glob
import os

package_name = 'radar_rviz'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for sub in ['launch','config','rviz']:
    if os.path.isdir(sub):
        data_files.append((os.path.join('share', package_name, sub), glob(sub + '/*')))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='yni5',
    maintainer_email='yni5@crimson.ua.edu',
    description='',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            
        ],
    },
)
