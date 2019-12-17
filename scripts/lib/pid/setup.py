#!/usr/bin/env python

"""
setup.py file for SWIG C\+\+/Python example
"""
from distutils.core import setup, Extension

example_module = Extension('_pid',
    sources=['pid_wrap.cxx',],
)
setup (
    name = 'pid',
    version = '0.1',
    author = "rpi",
    description = """Simple swig C\+\+/Python example""",
    ext_modules = [example_module],
    py_modules = ["pid"],
)
