
# from setuptools import setup

# package_name = 'sdformat_tools'

# setup(
#     name=package_name,
#     version='0.0.1',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     author='Zhenpeng Ge',
#     author_email='zhenpeng.ge@qq.com',
#     keywords=['ROS'],
#     classifiers=[
#         'Intended Audience :: Developers',
#         'Programming Language :: Python',
#         'Topic :: Software Development',
#     ],
#     description='sdformat tools.',
#     license='Apache-2.0',
#     entry_points={
#         'console_scripts': [
#             'xmacro4sdf = sdformat_tools.xmacro4sdf:xmacro4sdf_main',
#             'sdf2urdf = sdformat_tools.sdf2urdf:sdf2urdf_main',
#         ]
#     },
# )
from setuptools import setup
from setuptools.command.develop import develop as _develop


class ColconCompatibleDevelop(_develop):
    user_options = _develop.user_options + [
        ("editable", None, "compat option for colcon"),
        ("build-directory=", None, "compat option for colcon"),
    ]
    boolean_options = getattr(_develop, "boolean_options", []) + ["editable"]

    def initialize_options(self):
        super().initialize_options()
        self.editable = None
        self.build_directory = None

    def finalize_options(self):
        super().finalize_options()


package_name = "sdformat_tools"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="Zhenpeng Ge",
    author_email="zhenpeng.ge@qq.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="sdformat tools.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "xmacro4sdf = sdformat_tools.xmacro4sdf:xmacro4sdf_main",
            "sdf2urdf = sdformat_tools.sdf2urdf:sdf2urdf_main",
        ]
    },
    cmdclass={
        "develop": ColconCompatibleDevelop,
    },
)