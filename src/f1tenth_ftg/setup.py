from setuptools import setup

package_name = 'f1tenth_ftg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['f1tenth_ftg/config/ftg.yaml']),
        ('share/' + package_name + '/launch', ['launch/ftg.launch.py']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='FTG (Find The Gap) with LiDAR preprocessing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ftg_node = f1tenth_ftg.ftg_node:main',
        ],
    },
)
