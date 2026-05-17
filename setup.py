from setuptools import find_packages, setup
from glob import glob
import os


def package_dir_tree(target_dir, base_install_path):
    entries = {}
    for filepath in glob(os.path.join(target_dir, '**'), recursive=True):
        if os.path.isfile(filepath):
            relpath = os.path.relpath(filepath, start=target_dir)
            install_path = os.path.join(base_install_path, os.path.dirname(relpath))
            entries.setdefault(install_path, []).append(filepath)
    return list(entries.items())


package_name = 'ssc0712_t1'

data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
    (f'share/{package_name}/launch', glob('launch/*.py')),
    (f'share/{package_name}/description', glob('description/*.urdf.xacro')),
    (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
    (f'share/{package_name}/config', glob('config/*.yaml')),
]

if os.path.isdir('models'):
    data_files.extend(package_dir_tree('models', f'share/{package_name}/models'))
if os.path.isdir('world'):
    data_files.extend(package_dir_tree('world', f'share/{package_name}/world'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel Ferreira',
    maintainer_email='samuel.assuncao@usp.br',
    description='SSC0712 Trabalho 1: exploração, detecção visual de bandeira e captura com máquina de estados em ROS 2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ground_truth_odometry = ssc0712_t1.ground_truth_odometry:main',
            'robo_mapper = ssc0712_t1.robo_mapper:main',
            'flag_detector = ssc0712_t1.flag_detector:main',
            'mission_control = ssc0712_t1.mission_control:main',
        ],
    },
)
