build:
  symlink-install: true
  build-base: @(user_home_dir)/ros_ws/build
  install-base: @(user_home_dir)/ros_ws/install
  cmake-args:
    - -DCMAKE_BUILD_TYPE=RelWithDebInfo
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

test:
  build-base: @(user_home_dir)/ros_ws/build
  install-base: @(user_home_dir)/ros_ws/install
  log-base: @(user_home_dir)/ros_ws/logs
  event-handlers:
    - console_direct+

test-result:
  test-result-base: @(user_home_dir)/ros_ws/build

clean.workspace:
  'yes': true
  base-select:
    - build
    - install
    - log
    - test_result
  build-base: @(user_home_dir)/ros_ws/build
  install-base: @(user_home_dir)/ros_ws/install
  log-base: @(user_home_dir)/ros_ws/logs
  test-result-base: @(user_home_dir)/ros_ws/build

'':
  log-base: @(user_home_dir)/ros_ws/logs
