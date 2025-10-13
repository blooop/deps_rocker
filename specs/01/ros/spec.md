# ROS Extension Spec

- The ros extension must use the `apt_packages` feature to install ROS dependencies via apt.
- All required apt packages for ROS should be specified using this feature, not manual RUN commands.
- The implementation should leverage the built-in apt_packages logic for installation and caching.
- Dockerfile snippet should delegate package installation to apt_packages.
- Tests should verify apt_packages is used for ROS dependencies.
