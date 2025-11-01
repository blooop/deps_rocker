from setuptools import setup

package_name = "test_package"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Test User",
    maintainer_email="test@example.com",
    description="Test package used by the ROS Jazzy integration tests",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
