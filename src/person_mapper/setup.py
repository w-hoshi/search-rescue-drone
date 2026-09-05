from setuptools import setup

package_name = 'person_mapper'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='t248',
    maintainer_email='t248@email.com',
    description='Person mapper using YOLO and SLAM',
    license='MIT',
    entry_points={
        'console_scripts': [
            'person_mapper_node = person_mapper.person_mapper_node:main',
        ],
    },
)
