import os
import re
import sys
import shutil
import platform
import subprocess

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

YDLIDAR_SDK_REPO = "https://github.com/YDLIDAR/YDLidar-SDK"
YDLIDAR_SDK_BRANCH = "master"


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def clone(self):
        if not os.path.isdir("YDLidar-SDK"):
            subprocess.check_call('git clone {} -b {} --depth 1'.format(YDLIDAR_SDK_REPO, YDLIDAR_SDK_BRANCH).split())


    def build_extension(self, ext):
        #self.clone()
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir,
                      '-DPYTHON_EXECUTABLE=' + sys.executable]

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_{}={}'.format(cfg.upper(), extdir)]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_BUILD_TYPE=' + cfg]
            build_args += ['--', '-j2']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''),
                                                              self.distribution.get_version())
        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)
        print("build: ", self.build_temp)
        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd=self.build_temp, env=env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd=self.build_temp)
        shutil.copy(self.build_temp+"/python/ydlidar.py", extdir+"/")

setup(
    name='ydlidar',
    version='1.0.2',
    author='Tony',
    author_email='chushuifurong618@eaibot.com',
    url='https://github.com/YDLIDAR/YDLidar-SDK',
    description='YDLIDAR python SDK',
    long_description='',
    ext_modules=[CMakeExtension('ydlidar')],
    cmdclass=dict(build_ext=CMakeBuild),
    zip_safe=False,
)
