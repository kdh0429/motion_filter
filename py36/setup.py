from setuptools import find_packages, setup

setup(
    name="motion filter",
    version="1.0",
    description="Motion Filter",
    author="Donghyun Sung",
    author_email="dh-sung@naver.com",
    # install_requires=[git clone https://github.com/artivis/manif.git", "toml"],
    packages=find_packages(exclude=["docs", "tests*"]),
    keywords=["LieGroup"],
    python_requires=">=3.6",
)
