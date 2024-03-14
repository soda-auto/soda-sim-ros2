from setuptools import find_packages, setup

package_name = 'ackermann_drive_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/joyop.py', 'scripts/keyop.py']), 
        ('share/' + package_name + '/launch', 
         ['launch/ackermann_drive_joyop.launch.py', 'launch/ackermann_drive_keyop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alexthunderrex',
    maintainer_email='darkknightrises2018@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joyop = ackermann_drive_teleop.scripts.joyop:main', 
            'keyop = ackermann_drive_teleop.scripts.keyop:main',  
        ],
    },
)
