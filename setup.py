from setuptools import find_packages, setup

package_name = 'udp_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['models/VirtualSatv8.stl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='klingjac',
    maintainer_email='klingjac@umich.edu',
    description='Node for publishing Apriltag Pose Estimation to Rviz from an RPi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_listener_node = udp_listener.udp_listener_node:main'
        ],
    },
)
