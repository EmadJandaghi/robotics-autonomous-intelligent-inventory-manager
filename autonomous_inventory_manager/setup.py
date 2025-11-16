from setuptools import setup, find_packages
from glob import glob

package_name = 'autonomous_inventory_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/data', glob('data/*.*')),
        (f'share/{package_name}/launch', glob('launch/*.py')), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Autonomous inventory management project',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to_aoi = autonomous_inventory_manager.go_to_aoi:main',
            'object_detection = autonomous_inventory_manager.object_detection:main',
            'inventory_checking = autonomous_inventory_manager.inventory_checking:main',
            'autonomous_manager = autonomous_inventory_manager.autonomous_manager:main',
        ],
    },
)
