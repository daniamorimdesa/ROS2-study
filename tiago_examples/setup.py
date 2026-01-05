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
            "laser_tiago = tiago_examples.laser_tiago:main",
            "tiago_1 = tiago_examples.tiago_1:main", # adicionado entry point para tiago_1.py: código de odometria do TIAgo
            "tiago_2 = tiago_examples.tiago_2:main", # adicionado entry point para tiago_2.py: mover o TIAgo para uma posição desejada
            "tiago_3 = tiago_examples.tiago_3:main", 
            "tiago_obstacle_avoidance = tiago_examples.tiago_obstacle_avoidance:main",
        ],
    },
)
