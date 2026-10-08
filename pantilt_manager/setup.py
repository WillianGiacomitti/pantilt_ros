from setuptools import setup

package_name = 'pantilt_manager'

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
    description='Camada de decisão: inspection_manager (máquina de estados da inspeção).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspection_manager = pantilt_manager.inspection_manager:main',
        ],
    },
)
