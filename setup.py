from setuptools import find_packages, setup

package_name = 'ble_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='6kam',
    maintainer_email='6kam465@gmail.com',
    description='ROS 2 BLE Receiver Node for MIRS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ble_receiver_node = ble_server.ble_receiver_node:main',
        ],
    },
)
