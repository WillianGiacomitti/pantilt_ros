from setuptools import setup

package_name = 'pantilt_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Willian Luiz Giacomitti',
    maintainer_email='willian@todo.com',
    description='Acesso ao hardware do pan-tilt: ponte serial com a ESP32 e multiplexador de comandos.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge_node = pantilt_hardware.serial_bridge_node:main',
        ],
    },
)
