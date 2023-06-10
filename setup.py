from setuptools import find_packages, setup

package_name = 'zr30camera'
library_name = 'sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, library_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={package_name: ['siyi_sdk/*']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aimas',
    maintainer_email='aimas.lund@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = zr30camera.camera:main',
            'zoom = zr30camera.zoom:main',
            'gimbal = zr30camera.gimbal:main',
        ],
    },
)
