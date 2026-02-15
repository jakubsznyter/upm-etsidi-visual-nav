from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala los archivos de launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Instala archivos de config si los usas
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luis-daniel',
    maintainer_email='luis-daniel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_driver_node = robot_driver.robot_driver_node:main',
            'robot_tester_node = robot_driver.robot_test_node:main',
        ],
    },
)
