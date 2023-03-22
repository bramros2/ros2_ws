from setuptools import setup

package_name = 'pump_control'

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
    maintainer='bram',
    maintainer_email='bramo@live.nl',
    description='ROS2 control system for Ender3 syringe pumps',
    license='TODO: License declaration',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'flow_find  = pump_control.flow_node:main',
        	'point      = pump_control.point_node:main',
        	'mask       = pump_control.mask_node:main',
        	'keycontrol = pump_control.KeypressControl:main',
        	'teleoppump = pump_control.teleop_pumps:main',
        	'dino	    = pump_control.dinolite:main',
        	'drop_find  = pump_control.droplet_node:main'
        ],
    },
)
