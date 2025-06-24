from setuptools import setup

package_name = 'my_tf_rtabmap_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/start_with_tf.launch.py']),
        ('share/' + package_name + '/launch', ['launch/start_with_tf2.launch.py']),
        ('share/' + package_name + '/launch', ['launch/two_maps.launch.py']),
        ('share/' + package_name + '/launch', ['launch/two_maps.launchV2.py']),
        ('share/' + package_name + '/config', ['config/tf_transforms.yaml']),
        ('share/' + package_name + '/config', ['config/tf_transforms2.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dein_name',
    maintainer_email='dein@email.de',
    description='Startet TF + RTABMap mit Konfiguration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

