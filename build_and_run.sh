#! /usr/bin/env bash

function build_project(){
    mkdir -p build && cd build
    cmake ..
    make -j4
    cd ../
}

function run_project(){
    cd ./bin/main
    cd ../
}

function test_project(){
    cd gtest/
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
        clean)
            clean_project
            ;;
    esac
}

main "$@"
