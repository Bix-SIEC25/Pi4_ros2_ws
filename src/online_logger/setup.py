from setuptools import find_packages, setup

package_name = 'online_logger'

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
    maintainer='MagicTINTIN',
    maintainer_email='magictintin@proton.me',
    description='Logger that sends logs to server and receive data from websocket',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'online_logger = online_logger.LoggerSubscriber:main'
            'socket_listener = online_logger.SocketListener:main'
        ],
    },
)
