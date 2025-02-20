#!/usr/bin/env python

import argparse, os

os.environ["CC"] = "/scratch/weili3/gcc/bin/gcc"
os.environ["CXX"] = "/scratch/weili3/gcc/bin/g++"
os.environ["BOOST_ROOT"] = "/scratch/weili3/boost"
all_targets = ['route']
run_files = 'scripts/*.py eval drcu'

def run(command):
    if args.print_commands:
        print(command)
    if os.system(command) != 0:
        if not args.print_commands:
            print(command)
        quit()

# cmake & copy run files
# for sanitize, need to remove -static from src/CMakeLists.txt
mode_cmake_options = {
    None                : '',
    'debug'             : '-DCMAKE_BUILD_TYPE=Debug',
    'release'           : '-DCMAKE_BUILD_TYPE=Release',
    'profile'           : '-DCMAKE_CXX_FLAGS=-pg',
    'release_profile'   : '-DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-pg',
    'asan'              : '-DCMAKE_CXX_FLAGS=-fsanitize=address',
    'debug_asan'        : '-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-fsanitize=address',
    'tsan'              : '-DCMAKE_CXX_FLAGS=-fsanitize=thread',
    'debug_tsan'        : '-DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS=-fsanitize=thread'
}

# argparse
parser = argparse.ArgumentParser(description='Build CUGR in Linux')
parser.add_argument('-t', '--targets', choices=all_targets, nargs='+')
parser.add_argument('-o', '--mode', choices=mode_cmake_options.keys())
parser.add_argument('-c', '--cmake_options', default='')
parser.add_argument('-m', '--make_options', default='-j 6')
parser.add_argument('-p', '--print_commands', action='store_true')
parser.add_argument('-b', '--build_dir', default='build')
parser.add_argument('-r', '--run_dir', default='run')
args = parser.parse_args()

# targets
if args.targets is None:
    build_targets = ['']
else:
    build_targets = args.targets

run('/scratch/weili3/cmake-3.27.4-linux-x86_64/bin/cmake src -B {} {} {}'.format(args.build_dir, mode_cmake_options[args.mode], args.cmake_options))
run('mkdir -p {}'.format(args.run_dir))
run('cp -u -R {} {}'.format(run_files, args.run_dir))

# make
for target in build_targets:
    run('/scratch/weili3/cmake-3.27.4-linux-x86_64/bin/cmake --build {} -- {}'.format(args.build_dir, args.make_options))
cp_targets = all_targets if build_targets == [''] else build_targets
for target in cp_targets:
    run('cp -u {}/{} {}'.format(args.build_dir, target, args.run_dir))

# flute LUT
run('cp flute/*.dat {}'.format(args.run_dir))
