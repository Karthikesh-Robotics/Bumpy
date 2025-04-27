from setuptools import find_packages, setup

package_name = 'bumpy_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bumpy1',
    maintainer_email='bumpy1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_node = bumpy_sensors.camera:main",
            "oled_node = bumpy_sensors.oled:main",
            "imu_node = bumpy_sensors.imu:main"
        ],
    },
)
