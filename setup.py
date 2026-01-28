from setuptools import find_packages, setup

package_name = 'intro_robotics'

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
    maintainer='Vince Kurtz',
    maintainer_email='vkurtz1@depaul.edu',
    description='ROS2 package for CSE 375/475: Introduction to Robotics',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_world = intro_robotics.hello_world:main'
        ],
    },
)
