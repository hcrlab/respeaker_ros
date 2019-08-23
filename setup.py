from setuptools import find_packages
from setuptools import setup

setup(
    name='respeaker',
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[],
    install_requires=['setuptools'],
    author='Alexander Roessler',
    author_email='alex@machinekoder.com',
    maintainer='Alexander Roessler',
    maintainer_email='alex@machinekoder.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS interface for respeaker.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'respeaker_node = respeaker.respeaker_node:main',
        ],
    },
)
