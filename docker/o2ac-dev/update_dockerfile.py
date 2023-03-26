#!/usr/bin/env python3
import os
import apt
from pprint import pprint
from dockerfile_parse import DockerfileParser
from IPython.terminal.debugger import set_trace as keyboard
import numpy as np

# This script updates each library in the Dockerfile to the newest version.
# Each `apt-get install` command needs to be formatted with one library per line.

# Library versions can be obtained manually via the `apt-cache policy` command

# REQUIREMENTS:
# pip3 install dockerfile_parse
# apt-get install python3-apt

# =====

# colorful terminal output
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def update_libraries_without_vars(dockerfile_path):
    print("Update libraries with pinned versions")

    # load file separated in lines
    with open(dockerfile_path) as f:
        lines = f.readlines()

    # get the libraries installed with apt-get
    started = False
    libraries = []
    for line in lines:
        try:
            if '&& ' in line:
                started = False
            if 'RUN apt-get update' in line:
                started = True
            elif started:
                lib = line.rstrip()
                lib = lib.replace('\\', '')
                lib = lib.replace(' ', '')
                lib = lib.replace('\t', '')
                if lib[0] != '#':
                    libraries.append(lib)
        except Exception as e:
            print(e)
            print("Error while reading in lines. Are all libraries on their own line?")
            print("Failed on line:")
            print(line)

    # split into library names and versions
    lib_names, lib_versions = [], []
    for lib in libraries:
        if '=' in lib:
            lib_name_ver = lib.split('=')
            if lib_name_ver[1][0] != '$': # ignore variables for now
                lib_names.append(lib_name_ver[0])
                lib_versions.append(lib_name_ver[1])
        else:
            lib_names.append(lib)
            lib_versions.append('no specified')

    # update and open apt cache
    cache = apt.cache.Cache()
    print("Updating apt cache...")
    cache.update()
    print("Done")
    print("Opening apt cache...")
    cache.open()
    print("Done")

    # find the libraries to update
    libs_to_update = dict()
    for lib_name, docker_ver in zip(lib_names, lib_versions):
        pkg = cache.get(lib_name, None)
        if libs_to_update.get(lib_name):
            continue
        if not pkg:
            print(bcolors.WARNING + "[%s] WARNING: Package not found in cache, ignoring it." % str(lib_name) + bcolors.ENDC)
            continue
        if hasattr(pkg.installed, 'version'):
            if docker_ver != pkg.installed.version:
                print(bcolors.WARNING + "[%s] WARNING: Dockerfile version and installed version are different!" % str(lib_name) + bcolors.ENDC)
        if docker_ver != pkg.candidate.version:
            print(bcolors.FAIL + "[%s] UPDATED (old: %s --> new: %s)" % (str(lib_name), str(docker_ver), str(pkg.candidate.version)) + bcolors.ENDC)
            libs_to_update.update({lib_name: pkg.candidate.version})
        else:
            print("[%s] Up to date (%s)" % (str(lib_name), str(docker_ver)))

    # print('+++++',libs_to_update)

    # substitute the libraries versions

    newlines = []
    for line in lines:
        aux = False
        for lib in libs_to_update.items():
            old_line = str(line.replace(' ', '').replace('\\', '').replace('\t', '').strip())
            # if lib[0] in old_line:
            #     print('wdf', '|'+old_line+'|', 'lib', '|'+lib[0]+'|',lib[1],  '=' in old_line, lib[0] == old_line.split('=')[0], lib[0] == old_line )
            if ('=' in old_line and lib[0] == old_line.split('=')[0]) \
               or lib[0] == old_line:
                newline = '    '+str(lib[0])+'='+str(lib[1])+' \\\n'
                newlines.append(newline)
                aux = True
        if not aux:
            newlines.append(line)

    # update dockerfile
    print("Writing to file %s" % dockerfile_path + "...")
    with open(dockerfile_path, "w") as f:
        for line in newlines:
            f.write(str(line))
    print("Done")

def update_libraries_with_vars(dockerfile_path):
    print("Update libraries with variables")

    # load file separated in lines
    with open(dockerfile_path) as f:
        lines = f.readlines()

    # get the libraries installed with apt-get
    started = False
    libraries = []
    for line in lines:
        if '&& ' in line:
            started = False
        if 'RUN apt-get update' in line:
            started = True
        elif started:
            lib = line.rstrip()
            lib = lib.replace('\\', '')
            lib = lib.replace(' ', '')
            lib = lib.replace('\t', '')
            if lib[0] != '#':
                libraries.append(lib)

    # create a dockerfile parser
    dfp = DockerfileParser(dockerfile_path)

    # split into library names and versions
    lib_names, lib_versions, arg_names = [], [], []
    for lib in libraries:
        if '=' in lib: 
            lib_name_ver = lib.split('=')
            if lib_name_ver[1][0] == '$': # TODO this fails when the version is not pinned (i.e., no equal symbol present)
                lib_names.append(lib_name_ver[0])
                env_var_version = dfp.envs[lib_name_ver[1].replace('$', '').replace('{', '').replace('}', '')]
                lib_versions.append(env_var_version)
                arg_name = lib_name_ver[1].replace('$', '').replace('{', '').replace('}', '').lower()
                arg_names.append(arg_name)
        else:
            lib_names.append(lib)
            # env_var_version = dfp.envs[lib[1].replace('$', '').replace('{', '').replace('}', '')]
            env_var_version = 'None'
            lib_versions.append(env_var_version)
            # arg_name = lib[1].replace('$', '').replace('{', '').replace('}', '').lower()
            arg_name = 'None'
            arg_names.append(arg_name)

    # update and open apt cache
    cache = apt.cache.Cache()
    print("Updating apt cache...")
    cache.update()
    print("Done")
    print("Opening apt cache...")
    cache.open()
    print("Done")

    # find the libraries to update
    args_to_update = []
    for lib_name, docker_ver, arg_name in zip(lib_names, lib_versions, arg_names):
        pkg = cache[lib_name]
        if pkg.installed != None:
            if docker_ver != pkg.installed.version:
                print(bcolors.WARNING + "[%s] WARNING: Dockerfile version and installed version are different!" % str(lib_name) + bcolors.ENDC)
        if docker_ver != pkg.candidate.version:
            print(bcolors.FAIL + "[%s] UPDATED (old: %s --> new: %s)" % (str(lib_name), str(docker_ver), str(pkg.candidate.version)) + bcolors.ENDC)
            args_to_update.append([lib_name, pkg.candidate.version, arg_name])
        else:
            print("[%s] Up to date (%s)" % (str(lib_name), str(docker_ver)))

    # substitute the ARG directives to reflect the library versions
    newlines = []
    for line in lines:
        if 'ARG' in line[:3] and '=' in line:
            update = False
            for arg in args_to_update:
                print("checking against: ", arg)
                if arg[2] == line[4:].split('=')[0]:
                    newline = 'ARG '+str(arg[2])+'='+str(arg[1])+'\n'
                    newlines.append(newline)
                    update = True
                    print(bcolors.WARNING + "REPLACED")
                    break
            if not update:
                newlines.append(line)
        else:
            newlines.append(line)

    # update dockerfile
    print("Writing to file %s" % dockerfile_path + "...")
    with open(dockerfile_path, "w") as f:
        for line in newlines:
            f.write(str(line))
    print("Done")


def update_libraries(dockerfile_path):
    update_libraries_without_vars(dockerfile_path)
    # update_libraries_with_vars(dockerfile_path)


if __name__ == '__main__':
    dockerfile_path = "./Dockerfile"
    update_libraries(dockerfile_path)
    print("Finished")
