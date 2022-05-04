from setuptools import setup

package_name = 'robVis_pkg'

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
    maintainer='Ben',
    maintainer_email='bomh19@student.aau.dk',
    description='Publish Aruco markers transform',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robVis_node = robVis_pkg.robVis_node:main'
        ],
    },
)
