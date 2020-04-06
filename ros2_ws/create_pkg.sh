#!/usr/bin/env bash
set -e

usage()
{
    echo "usage: create_pkg -n package_name [-l(ibrary)]"
}

# Handle options
is_library=
package_name="test_package"

while [ "$1" != "" ]; do
    case $1 in
        -n | --name )           shift
                                package_name=$1
                                ;;
        -l | --library )        is_library=1
                                ;;
        -h | --help )           usage
                                exit
                                ;;
        * )                     usage
                                exit 1
    esac
    shift
done

description="boilerplate"
license="MIT"
dest="${PWD}/src"
build_type="ament_cmake"
email="charles@missionrobotics.us"
name="Charles Cross"

if [ "$is_library" = "1" ]; then
    echo "Creating Library"
    ros2 pkg create \
        --description "${description}" \
        --destination-directory "${dest}" \
        --build-type "${build_type}" \
        --maintainer-email "${email}" \
        --maintainer-name "${name}" \
        --cpp-library-name "${package_name}" \
        "${package_name}"
else
    echo "Creating Node"
    ros2 pkg create \
        --description "${description}" \
        --destination-directory "${dest}" \
        --build-type "${build_type}" \
        --maintainer-email "${email}" \
        --maintainer-name "${name}" \
        --cpp-node-name "${package_name}" \
        "${package_name}"
fi
