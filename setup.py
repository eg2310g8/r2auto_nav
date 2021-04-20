from setuptools import setup

package_name = 'auto_nav'

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
    maintainer='nus',
    maintainer_email='nus@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2mover = auto_nav.Original_Files.r2mover:main',
            'r2moverotate = auto_nav.Original_Files.r2moverotate:main',
            'r2scanner = auto_nav.Original_Files.r2scanner:main',
            'r2occupancy = auto_nav.Original_Files.r2occupancy:main',
            'r2occupancy2 = auto_nav.Original_Files.r2occupancy2:main',
            'r2auto_nav = auto_nav.Partially_Working_Navigation.r2auto_nav:main',
            'r2auto_navtrial = auto_nav.Partially_Working_Navigation.r2auto_navtrial:main',
            'autonav = auto_nav.Partially_Working_Navigation.r2_corner_based_navigation:main',
            'mywall = auto_nav.Partially_Working_Navigation.wall_follower_alissacopy:main',
            'r2wallfollower = auto_nav.r2wallfollower:main',
            'r2targeting = auto_nav.r2targeting:main',
        ],
    },
)
