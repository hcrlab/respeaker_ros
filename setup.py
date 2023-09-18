from setuptools import find_packages
from setuptools import setup

package_name = 'respeaker_ros'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name])],
    py_modules=[],
    install_requires=['setuptools'],
    author='Nick Walker',
    author_email='nswalker@cs.washington.edu',
    maintainer='Nick Walker',
    maintainer_email='nswalker@cs.washington.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS interface for respeaker.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'respeaker_node = respeaker_ros.respeaker_node:main',
        ],
    },
)
