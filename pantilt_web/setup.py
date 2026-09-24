import os
from glob import glob

from setuptools import setup

package_name = 'pantilt_web'


def web_data_files():
    """Instala a árvore web/ em share/pantilt_web/web, preservando as subpastas."""
    files = []
    for root, _, names in os.walk('web'):
        paths = [os.path.join(root, n) for n in names]
        if paths:
            files.append((os.path.join('share', package_name, root), paths))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ] + web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Willian Luiz Giacomitti',
    maintainer_email='willian@todo.com',
    description='Interface web do operador do pan-tilt: página HTTP, rosbridge e web_video_server.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
