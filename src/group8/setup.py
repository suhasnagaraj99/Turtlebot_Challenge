from setuptools import find_packages, setup

package_name = 'group8'

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
    maintainer='suhas99',
    maintainer_email='suhas99@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'haar = group8.haar_cascade:main',
            'yolo = group8.yolo:main',
            'optical = group8.optical_flow_interface:main',
            'detect_horizon = group8.detect_horizon:main',
            'turtlebot_controller = group8.turtlebot_controller:main',
            'paper_centroid = group8.paper_centroid:main',
        ],
    },
)