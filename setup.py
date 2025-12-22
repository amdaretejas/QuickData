from setuptools import setup, find_packages

setup(
    name="QuickData",
    version="1.0.0",
    author="Tejas Amdare",
    author_email="amdaretejas1@gmail.com",
    description="This is a simple lib to store and get data in three simple steps",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
)
