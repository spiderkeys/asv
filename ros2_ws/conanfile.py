from conans import ConanFile, CMake, tools, RunEnvironment
import os

class VoyageExampleConan( ConanFile ):
    name            = "nadir_os"
    version         = "1.0.0"
    author          = "charles@missionrobotics.us"
    description     = ""
    license         = "Proprietary"

    settings        = "os", "compiler", "arch", "build_type"
    options         = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": False, "fPIC": True}
    generators      = "cmake"

    requires =  (
                    ( "Catch2/2.5.0@catchorg/stable" ),
                    ( "spdlog/1.3.1@bincrafters/stable" ),
                    ( "boost/1.71.0@conan/stable" ),
                    ( "mavchannel/1.0.0@missionrobotics/stable" ),
                    ( "libjpeg-turbo/2.0.4@" )
                )