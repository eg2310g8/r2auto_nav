from setuptools import setup

package_name = 'r2auto_nav'

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
            'r2mover = r2auto_nav.Original_Files.r2mover:main',
            'r2moverotate = r2auto_nav.Original_Files.r2moverotate:main',
            'r2scanner = r2auto_nav.Original_Files.r2scanner:main',
            'r2occupancy = r2auto_nav.Original_Files.r2occupancy:main',
            'r2occupancy2 = r2auto_nav.Original_Files.r2occupancy2:main',
            'dtnav = r2auto_nav.dtnav:main',
            'trinav = r2auto_nav.trinav:main',
            'tri2nav = r2auto_nav.tri2nav:main',
            'frontier = r2auto_nav.mapping:main'
        ],
    },
)
