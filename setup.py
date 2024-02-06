from setuptools import setup, find_packages

setup(
    name="driving_analysis",
    version="0.1.0",
    packages=find_packages(exclude=["test", "driving_data"]),
    install_requires=[
        # future dependencies
    ],
)
