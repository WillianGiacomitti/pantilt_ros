from setuptools import setup

package_name = 'pantilt_control'

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
    description='Controle do pan-tilt: varredura (scan_node) e servo visual IBVS (visual_servo_node).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_node = pantilt_control.scan_node:main',
        ],
    },
)
