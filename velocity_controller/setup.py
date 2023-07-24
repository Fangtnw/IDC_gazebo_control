from setuptools import setup

package_name = 'velocity_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config/key_config.yaml']),
        ('share/' + package_name, ['config/connect-server.bash']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fang',
    maintainer_email='thanawat.smkn@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "velocity_controller = velocity_controller.velocity_controller:main",
        ],
    },
)
