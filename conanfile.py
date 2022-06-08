# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "traact_component_kinect_azure"
    description = "Traact Kinect Azure driver component"
    url = "https://github.com/traact/traact_component_kinect_azure.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    def _options(self):
        self.options["with_bodytracking"] = [True, False]
        self.default_options["with_bodytracking"] = True

    exports_sources = "src/*", "util/*", "tests/*", "CMakeLists.txt"

    def requirements(self):
        self.traact_requires("traact_vision", "latest")
        self.traact_requires("traact_spatial", "latest")
        self.requires("kinect-azure-sensor-sdk/1.4.1@camposs/stable")
        if self.options.with_bodytracking:
            self.requires("kinect-azure-bodytracking-sdk/1.1.0@vendor/stable")
        if self.options.with_tests:
            self.requires("gtest/[>=1.11.0]")