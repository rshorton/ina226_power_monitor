from setuptools import setup

package_name = 'ina226_power_monitor'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Scott Horton',
    maintainer_email='none@none.com',
    description='Power monitor using TI INA226 Power Monitor IC',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ina226_power_monitor = ina226_power_monitor.ina226_power_monitor:main',
        ],
    },
)
