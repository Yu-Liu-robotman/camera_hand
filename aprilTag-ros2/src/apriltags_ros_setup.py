from setuptools import setup, find_packages

setup(
    name="apriltags_ros",
    version="0.0.1",
    packages=find_packages(include=['apriltags_ros*']),
    install_requires=[
        'numpy',
        'opencv-python',
        'PyYAML',
    ],
)
