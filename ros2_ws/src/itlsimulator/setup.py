from setuptools import setup, find_packages

package_name = 'itlsimulator'
package_version = '0.0.3'

setup(
    name=package_name,
    version=package_version,
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'ui_files/*.ui',
            'ItlApiCode/*.py',
            'itlsimulator/ItlApiCode/*/*.py',
            'ItlComProtocol/*.py'
            'itlsimulator/ItlComProtocol/*/*.py'
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name, [
            'package.xml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'pyqt5',
        'rclpy',
        'rclpy_action',
        'rclpy_components',
        'rclpy_logging',
        'rclpy_parameter',
        'rclpy_service',
        'rclpy_task',
        'rclpy_tutorial'
    ],
    zip_safe=True,
    author='Andrei C',
    author_email='crisu.radu.m4e@student.ucv.ro',
    maintainer='Andrei C',
    maintainer_email='crisu.radu.m4e@student.ucv.ro',
    description='A package for demonstrating ROS2 pub-sub using PyQt5',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = itlsimulator.pubGuiNode:main',
            'listener = itlsimulator.subGuiNode:main',
            'stalker = itlsimulator.simplepublisher:main',
            'slistener = itlsimulator.simplesubscriber:main',
            'itlsubgui = itlsimulator.itl_subscriber:main',
            'itlpubgui = itlsimulator.itl_publisher:main',
            'getterUi = itlsimulator.itlGetterUi:main',
            'setterUi = itlsimulator.itlSetterUi:main'
        ]
    }
)
