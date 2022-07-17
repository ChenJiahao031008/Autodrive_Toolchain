#! /usr/bin/env bash

function build_project(){
    mkdir -p build && cd build
    cmake ..
    make -j4
    cd ../
}

function run_project(){
    cd bin/
    ./main
    cd ../
}

function test_project(){
    cd build/gtest/
    ./senser_data_test
    ./interface_test
    ./common_test
    ./math_utils_test
    ./math_kalman_filter_test
    ./third_party_test
    cd ../
}

function clean_project(){
    rm -rf log/
    rm -rf build/*
    rm -rf bin/*
    rm -rf gtest/results/
    rm -rf gtest/log/
    rm -rf ros_interface/build/
}


function build_ros_project(){
    cd ros_interface/
    mkdir -p build && cd build
    cmake ..
    make -j4
    cd ../../
}

function run_ros_project(){
    cd ros_interface/build/
    source devel/setup.bash
    roslaunch ros_interface ros_test.launch
    cd ../../
}

function ros_save_traj(){
    cd ros_interface/build/
    source devel/setup.bash
    rosservice call /save_odometry "{}"
    cd ../../
}


function main() {
    local cmd="$1"
    shift
    case "${cmd}" in
        build)
            build_project
            ;;
        test)
            test_project
            ;;
        run)
            run_project
            ;;
        build_and_test)
            build_project && test_project
            ;;
        build_and_run)
            build_project && run_project
            ;;
        ros_build)
            build_ros_project
            ;;
        ros_run)
            run_ros_project
            ;;
        ros_build_and_run)
            build_ros_project && run_ros_project
            ;;
        ros_save_traj)
            ros_save_traj
            ;;
        clean)
            clean_project
            echo "clean_project finished."
            ;;
        *)
            echo "${cmd} Unrecognized"
            ;;

    esac
}

main "$@"
