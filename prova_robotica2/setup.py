from setuptools import find_packages, setup

package_name = 'prova_robotica2'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "number_publisher_simple = prova_robotica2.number_publisher_simple:main",
            "number_counter_simple = prova_robotica2.number_counter_simple:main",
            "set_speed_server = prova_robotica2.set_speed_server:main",
            "set_speed_client = prova_robotica2.set_speed_client:main",
        ],
    },
)
