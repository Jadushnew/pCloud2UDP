from setuptools import find_packages, setup

package_name = 'pCloud2UDP'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/config', ['config/config.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lennart',
    maintainer_email='lennart.weinstock@student.kit.edu',
    description='Subscribes to /unilidar/cloud publishes the messages via UDP',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pCloud2UDP_node = pCloud2UDP.pCloud2UDP:main'
        ],
    },
)
