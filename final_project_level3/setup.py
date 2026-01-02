from setuptools import find_packages, setup

package_name = 'final_project_level3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniela',
    maintainer_email='daniela.amorimdesa@gmail.com',
    description='Final project level 3 with turtlesim',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'turtle_controller = final_project_level3.turtle_controller:main',
            'turtle_controller_2 = final_project_level3.turtle_controller_2:main',
            "turtle_controller_3 = final_project_level3.turtle_controller_3:main",
            
        ],
    },
)
