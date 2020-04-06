set -e


conan create ./conan-bzip2 conan/stable
conan install boost/1.71.0@conan/stable --build missing
conan create ./conan-concurrentqueue missionrobotics/stable
conan create ./conan-readerwriterqueue missionrobotics/stable
conan create ./conan-serial missionrobotics/stable
conan create ./mavlink2 missionrobotics/stable
conan create ./mavchannel missionrobotics/stable
