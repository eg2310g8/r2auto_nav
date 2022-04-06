from setuptools import setup

package_name = 'noob_nav'

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
            'r2mover = noob_nav.Original_Files.r2mover:main',
            'r2moverotate = noob_nav.Original_Files.r2moverotate:main',
            'r2scanner = noob_nav.Original_Files.r2scanner:main',
            'r2occupancy = noob_nav.Original_Files.r2occupancy:main',
            'r2occupancy2 = noob_nav.Original_Files.r2occupancy2:main',
            'r2noob = noob_nav.r2noob:main',
            'r2triangle = noob_nav.r2triangle:main'
        ],
    },
)
