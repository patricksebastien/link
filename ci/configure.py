#!/usr/bin/env python

import argparse
import logging
import os
import shutil
import sys

from distutils.spawn import find_executable
from subprocess import call


def parse_args():
    arg_parser = argparse.ArgumentParser()

    arg_parser.add_argument(
        '-a', '--audio-driver',
        default='Asio',
        help='Audio driver to build (Windows only, default: %(default)s)')

    arg_parser.add_argument(
        '--cmake',
        default=find_executable("cmake"),
        help='Path to CMake executable (default: %(default)s)')

    arg_parser.add_argument(
        '-c', '--configuration',
        help='Build configuration to use (not supported by IDE generators)')

    arg_parser.add_argument(
        '-q', '--with-qt',
        help='Build Qt example apps',
        action='store_true')

    arg_parser.add_argument(
        '-g', '--generator',
        help='CMake generator to use (default: Determined by CMake)')

    arg_parser.add_argument(
        '-w', '--wordsize', type=int, default='64',
        help='Set word size for build (must be either 32/64, default: %(default)s)')

    return arg_parser.parse_args(sys.argv[1:])


def build_cmake_args(args):
    if args.cmake is None:
        logging.error('CMake not found, please use the --cmake option')
        return None

    cmake_args = []
    cmake_args.append(args.cmake)

    if args.generator is not None:
        cmake_args.append('-G')
        cmake_args.append(args.generator)

    if args.with_qt:
        cmake_args.append('-DLINK_BUILD_QT_EXAMPLES=ON')

    if args.wordsize is not None:
        cmake_args.append('-DLINK_WORD_SIZE=' + str(args.wordsize))

    if args.configuration is not None:
        cmake_args.append('-DCMAKE_BUILD_TYPE=' + args.configuration)

    if sys.platform == 'win32':
        if args.audio_driver is None or args.audio_driver == 'Asio':
            cmake_args.append('-DLINK_BUILD_ASIO=ON')
        else:
            cmake_args.append('-DLINK_BUILD_ASIO=OFF')

    # This must always be last
    cmake_args.append('..')
    return cmake_args


def configure(args):
    scripts_dir = os.path.dirname(os.path.realpath(__file__))
    root_dir = os.path.join(scripts_dir, os.pardir)
    build_dir = os.path.join(root_dir, 'build')
    if os.path.exists(build_dir):
        logging.info('Removing existing build directory')
        shutil.rmtree(build_dir)

    logging.debug('Creating build directory')
    os.mkdir(build_dir)
    os.chdir(build_dir)

    cmake_args = build_cmake_args(args)
    if cmake_args is None:
        return 1

    logging.info('Running CMake')
    return call(cmake_args)


if __name__ == '__main__':
    logging.basicConfig(format='%(message)s', level=logging.INFO, stream=sys.stdout)
    sys.exit(configure(parse_args()))