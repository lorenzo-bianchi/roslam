from setuptools import setup
import os
from glob import glob

package_name = 'uwb_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lorenzo Bianchi',
    maintainer_email='lnz.bnc@gmail.com',
    description='UWB driver module.',
    license='GNU GPL v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uwb_driver = uwb_driver.uwb_driver:main',
        ],
    },
)
