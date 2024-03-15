from setuptools import setup

package_name = 'lidar_subscriber'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Lidar subscriber: Processes data from rplidar_ros/scan',
    license='Apache 2.0: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'lidar_subscriber = lidar_subscriber.lidar_subscriber:main',
        ],
    },
)
