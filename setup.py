from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup

package_name = 'pendulum_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf/pendulum'), glob('urdf/pendulum/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agillies8',
    maintainer_email='andrew.gillies@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cart_pub = pendulum_sim.cart_pub:main',
            'theta_plotter = pendulum_sim.theta_plotter:main',
            'lqr_controller = pendulum_sim.lqr_controller:main',
            'state_pub = pendulum_sim.state_pub:main',

        ],
    },
)
