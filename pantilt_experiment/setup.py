from setuptools import setup

package_name = 'pantilt_experiment'

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
    description='Ensaios do TCC: gravação de cada inspeção em CSV e JSON, com snapshots ou vídeo opcionais (experiment_recorder), e análise com métricas e gráficos (analisar_ensaios).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'experiment_recorder = pantilt_experiment.experiment_recorder:main',
            'analisar_ensaios = pantilt_experiment.analisar_ensaios:main',
        ],
    },
)
