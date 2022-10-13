import re
from os import path

from setuptools import setup, find_packages

# read the contents of README file
this_directory = path.abspath(path.dirname(__file__))
with open(path.join(this_directory, "README.md"), encoding="utf-8") as f:
    long_description = f.read()

# read the version file
VERSIONFILE = "sbp_env/_version.py"
verstrline = open(VERSIONFILE, "rt").read()
mo = re.search(r"^__version__ = ['\"]([^'\"]*)['\"]", verstrline, re.M)
if not mo:
    raise RuntimeError("Unable to find version string in %s." % (VERSIONFILE,))
version_str = mo.group(1)
with open("requirements.txt") as f:
    install_requires = [l for l in f.read().splitlines() if l]

setup(
    name="sbp-env",
    version=version_str,
    description="Motion planning environment for Sampling-based Planners",
    long_description=long_description,
    long_description_content_type="text/markdown",
    author="Tin Lai (@soraxas)",
    author_email="oscar@tinyiu.com",
    license="MIT",
    url="https://github.com/soraxas/sbp-env",
    keywords="motion planning robotics sbp rrt",
    python_requires=">=3.7",
    # packages=find_packages(where="sbp_env"),
    packages=[
        "sbp_env",
        "sbp_env.planners",
        "sbp_env.samplers",
        "sbp_env.utils",
    ],
    install_requires=install_requires,
    entry_points={
        "console_scripts": [
            "sbp-env=sbp_env:run_cli",
        ]
    },
    classifiers=[
        "Environment :: Console",
        "Framework :: Matplotlib",
        "Intended Audience :: Developers",
        "Intended Audience :: End Users/Desktop",
        "Intended Audience :: System Administrators",
        "License :: OSI Approved :: MIT License",
        "Operating System :: MacOS",
        "Operating System :: POSIX",
        "Operating System :: Unix",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Topic :: Desktop Environment",
        "Topic :: Terminals",
        "Topic :: Utilities",
    ],
)
