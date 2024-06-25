# /usr/bin/python3
import os
from conan import ConanFile
from conan.tools.build import can_run

class TraactPackage(ConanFile):
    python_requires = "traact_base/0.0.0@traact/latest"
    python_requires_extend = "traact_base.TraactPackageCmake"

    name = "traact_component_kinect_azure"
    version = "0.0.0"
    description = "Traact Kinect Azure driver component"
    url = "https://github.com/traact/traact_component_kinect_azure.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "src/*", "util/*", "tests/*", "CMakeLists.txt"

    options = {
        "shared": [True, False],
        "trace_logs_in_release": [True, False],
        "with_bodytracking": [True, False]
    }

    default_options = {
        "shared": True,
        "trace_logs_in_release": True,
        "with_bodytracking": True
    }

    def requirements(self):
        self.requires("traact_spatial/0.0.0@traact/latest")
        self.requires("traact_vision/0.0.0@traact/latest")
        self.requires("kinect-azure-sensor-sdk/1.4.1-r3@camposs/stable", run=True)
        if self.options.with_bodytracking:
            self.requires("kinect-azure-bodytracking-sdk/1.1.0@vendor/stable", run=True)

    def _after_package_info(self):
        self.cpp_info.libs = ["traact_component_kinect_azure"]