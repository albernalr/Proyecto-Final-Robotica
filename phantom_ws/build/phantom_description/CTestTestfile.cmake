# CMake generated Testfile for 
# Source directory: /home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description
# Build directory: /home/alejandro/ros2_ws/phantom_ws/build/phantom_description
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(flake8 "/home/alejandro/.local/share/mamba/envs/ros_env/bin/python3" "-u" "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/run_test.py" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/flake8.xunit.xml" "--package-name" "phantom_description" "--output-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/ament_flake8/flake8.txt" "--command" "/home/alejandro/.local/share/mamba/envs/ros_env/bin/ament_flake8" "--xunit-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/flake8.xunit.xml")
set_tests_properties(flake8 PROPERTIES  LABELS "flake8;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description" _BACKTRACE_TRIPLES "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_flake8/cmake/ament_flake8.cmake;63;ament_add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;18;ament_flake8;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_flake8/cmake/ament_cmake_flake8_lint_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;34;ament_package;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;0;")
add_test(lint_cmake "/home/alejandro/.local/share/mamba/envs/ros_env/bin/python3" "-u" "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/run_test.py" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/lint_cmake.xunit.xml" "--package-name" "phantom_description" "--output-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/ament_lint_cmake/lint_cmake.txt" "--command" "/home/alejandro/.local/share/mamba/envs/ros_env/bin/ament_lint_cmake" "--xunit-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/lint_cmake.xunit.xml")
set_tests_properties(lint_cmake PROPERTIES  LABELS "lint_cmake;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description" _BACKTRACE_TRIPLES "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_lint_cmake/cmake/ament_lint_cmake.cmake;47;ament_add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;21;ament_lint_cmake;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_lint_cmake/cmake/ament_cmake_lint_cmake_lint_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;34;ament_package;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;0;")
add_test(pep257 "/home/alejandro/.local/share/mamba/envs/ros_env/bin/python3" "-u" "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/run_test.py" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/pep257.xunit.xml" "--package-name" "phantom_description" "--output-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/ament_pep257/pep257.txt" "--command" "/home/alejandro/.local/share/mamba/envs/ros_env/bin/ament_pep257" "--xunit-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/pep257.xunit.xml")
set_tests_properties(pep257 PROPERTIES  LABELS "pep257;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description" _BACKTRACE_TRIPLES "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_pep257/cmake/ament_pep257.cmake;41;ament_add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;18;ament_pep257;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_pep257/cmake/ament_cmake_pep257_lint_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;34;ament_package;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;0;")
add_test(xmllint "/home/alejandro/.local/share/mamba/envs/ros_env/bin/python3" "-u" "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/run_test.py" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/xmllint.xunit.xml" "--package-name" "phantom_description" "--output-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/ament_xmllint/xmllint.txt" "--command" "/home/alejandro/.local/share/mamba/envs/ros_env/bin/ament_xmllint" "--xunit-file" "/home/alejandro/ros2_ws/phantom_ws/build/phantom_description/test_results/phantom_description/xmllint.xunit.xml")
set_tests_properties(xmllint PROPERTIES  LABELS "xmllint;linter" TIMEOUT "60" WORKING_DIRECTORY "/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description" _BACKTRACE_TRIPLES "/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_xmllint/cmake/ament_xmllint.cmake;50;ament_add_test;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;18;ament_xmllint;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_xmllint/cmake/ament_cmake_xmllint_lint_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;21;ament_execute_extensions;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_lint_auto/cmake/ament_lint_auto_package_hook.cmake;0;;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_execute_extensions.cmake;48;include;/home/alejandro/.local/share/mamba/envs/ros_env/share/ament_cmake_core/cmake/core/ament_package.cmake;66;ament_execute_extensions;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;34;ament_package;/home/alejandro/ros2_ws/phantom_ws/src/phantomx-workbench/phantom_description/CMakeLists.txt;0;")
