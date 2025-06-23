from setuptools import find_packages, setup

package_name = 'kamerabild'

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
    maintainer='benutzer',
    maintainer_email='benutzer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'kamerabild1 = kamerabild.kamerabild1:main',
        'lidar1 = kamerabild.lidar1:main',
        'testScan = kamerabild.testScan:main',
        'lidar5006 = kamerabild.lidar5006:main',
        'MultiBoolRelayNode = kamerabild.MultiBoolRelayNode:main',
        'MultiBoolTestNode = kamerabild.MultiBoolTestNode:main'
        ],
    },
)
