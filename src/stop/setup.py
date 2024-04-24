from setuptools import find_packages, setup

package_name = 'stop'

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
            'test = stop.test:main',
            'test2 = stop.test2:main',
            'test3 = stop.test3:main',
            'test5 = stop.test5:main',
            'test6 = stop.test6:main',
        ],
    },
)
