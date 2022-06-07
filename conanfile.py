# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class Traact(ConanFile):
    name = "traact_component_kinect_azure"
    version = "0.1.0"

    description = "Traact Kinect Azure driver component"
    url = ""
    license = "MIT"
    author = "Frieder Pankratz"

    short_paths = True

    generators = "cmake"
    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"
    options = {
        "shared": [True, False],
        "with_bodytracking": [True, False],
        "with_tests": [True, False]
    }

    default_options = {
        "shared": True,
        "with_tests": True,
        "with_bodytracking": True
    }

    exports_sources = "src/*", "util/*", "tests/*", "CMakeLists.txt"

    def requirements(self):
        self.requires("traact_vision/[>=0.1.0]@camposs/stable")
        self.requires("traact_spatial/[>=0.1.0]@camposs/stable")

        self.requires("kinect-azure-sensor-sdk/1.4.1@camposs/stable")
        if self.options.with_bodytracking:
            self.requires("kinect-azure-bodytracking-sdk/1.1.0@vendor/stable")

        if self.options.with_tests:
            self.requires("gtest/1.10.0")

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.verbose = True

        def add_cmake_option(option, value):
            var_name = "{}".format(option).upper()
            value_str = "{}".format(value)
            var_value = "ON" if value_str == 'True' else "OFF" if value_str == 'False' else value_str
            cmake.definitions[var_name] = var_value

        for option, value in self.options.items():
            add_cmake_option(option, value)

        cmake.configure()
        return cmake

    def configure(self):
        self.options['traact_vision'].shared = self.options.shared
        self.options['traact_spatial'].shared = self.options.shared

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = [self.name]
        # self.cpp_info.libs = tools.collect_libs(self)
