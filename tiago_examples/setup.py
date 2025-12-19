from setuptools import find_packages, setup

package_name = 'tiago_examples'

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
    maintainer='das9',
    maintainer_email='daniela.amorimdesa@gmail.com',
    description='Exemplos de código para o robô TIAgo desenvolvidos em sala de aula',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "tiago = tiago_examples.tiago:main",
            "tiago2 = tiago_examples.tiago2:main",
            "laser_tiago = tiago_examples.laser_tiago:main",
        ],
    },
)
