%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-autoware-lanelet2-extension-python
Version:        0.5.0
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS autoware_lanelet2_extension_python package

License:        Apache License 2.0
Source0:        %{name}-%{version}.tar.gz

Requires:       boost-python3-devel
Requires:       ros-humble-autoware-lanelet2-extension
Requires:       ros-humble-geometry-msgs
Requires:       ros-humble-lanelet2-core
Requires:       ros-humble-lanelet2-io
Requires:       ros-humble-lanelet2-projection
Requires:       ros-humble-lanelet2-python
Requires:       ros-humble-lanelet2-routing
Requires:       ros-humble-lanelet2-traffic-rules
Requires:       ros-humble-lanelet2-validation
Requires:       ros-humble-rclcpp
Requires:       ros-humble-ros-workspace
BuildRequires:  boost-python3-devel
BuildRequires:  ros-humble-ament-cmake-auto
BuildRequires:  ros-humble-autoware-cmake
BuildRequires:  ros-humble-autoware-lanelet2-extension
BuildRequires:  ros-humble-geometry-msgs
BuildRequires:  ros-humble-lanelet2-core
BuildRequires:  ros-humble-lanelet2-io
BuildRequires:  ros-humble-lanelet2-projection
BuildRequires:  ros-humble-lanelet2-python
BuildRequires:  ros-humble-lanelet2-routing
BuildRequires:  ros-humble-lanelet2-traffic-rules
BuildRequires:  ros-humble-lanelet2-validation
BuildRequires:  ros-humble-python-cmake-module
BuildRequires:  ros-humble-rclcpp
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  ros-humble-ament-cmake-ros
%endif

%description
The autoware_lanelet2_extension_python package contains Python bindings for
lanelet2_extension package

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Wed Jul 10 2024 Mamoru Sobue <mamoru.sobue@tier4.jp> - 0.5.0-1
- Autogenerated by Bloom

* Mon Jun 24 2024 Mamoru Sobue <mamoru.sobue@tier4.jp> - 0.4.0-1
- Autogenerated by Bloom

