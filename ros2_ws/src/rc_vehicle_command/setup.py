from setuptools import find_packages, setup

package_name = 'rc_vehicle_command'

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
    maintainer='vscode',
    maintainer_email='grf317@gmail.com',
    description='Vehicle command adapter for a bicycle/Ackermann-type RC vehicle',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vehicle_command_adapter = rc_vehicle_command.vehicle_command_adapter:main',
        ],
    },
)
