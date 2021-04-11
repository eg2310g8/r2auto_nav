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
            'r2mover = auto_nav.r2mover:main',
            'r2moverotate = auto_nav.r2moverotate:main',
            'r2scanner = auto_nav.r2scanner:main',
            'r2occupancy = auto_nav.r2occupancy:main',
            'r2occupancy2 = auto_nav.r2occupancy2:main',
            'r2auto_nav = auto_nav.r2auto_nav:main',
            'r2homework = auto_nav.r2homework:main',
            'r2auto_navtrial = auto_nav.r2auto_navtrial:main',
            'orgybaby = auto_nav.r2auto_navtrial_original:main',
            'mapping = auto_nav.r2_getmapinfo:main',
            'navi = auto_nav.r2_dothenavigation:main',
            'autonav = auto_nav.r2auto_navtrial_original_alissacopy:main',
            'wallfollower = auto_nav.wall_follower:main',
            'mywall = auto_nav.wall_follower_alissacopy:main',
            'posing = auto_nav.pose_estimator:main',
            'bugging = auto_nav.bugging:main',
            'goalie = auto_nav.goal_giver:main',
            'targeting = auto_nav.r2_targeting:main',
        ],
    },
)
