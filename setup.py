from setuptools import setup

with open("README.md", "r") as f:
    long_description = f.read()

setup(
    name='deps_rocker',
    version='0.0.2',
    packages=['deps_rocker'],
    package_data={'deps_rocker': ['templates/*.em']},
    author='Austin Gregg-Smith',
    author_email='blooop@gmail.com',
    description='A rocker plugin to help installing apt and pip dependencies',
    long_description=long_description,
    long_description_content_type="text/x-rst",
    url="https://github.com/blooop/deps_rocker",
    license='MIT',
    install_requires=[
        'rocker',
    ],
    entry_points={
        'rocker.extensions': [
            'deps_deps = deps_rocker.dependencies:Dependencies',
        ]
    },
)
