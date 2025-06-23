from setuptools import setup

package_name = 'map_sector_monitor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lorenz',
    maintainer_email='lorenz@example.com',
    description='Map-based sector collision monitor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'map_sector_monitor = map_sector_monitor.map_sector_monitor:main',
            'map_sector_monitor_Kreis_drehen = map_sector_monitor.map_sector_monitor_Kreis_drehen:main',
            'map_sector_monitor_rechteck = map_sector_monitor.map_sector_monitor_rechteck:main',
            'map_sector_monitor_rechteck_rviz = map_sector_monitor.map_sector_monitor_rechteck_rviz:main'
        ],
    },
)


