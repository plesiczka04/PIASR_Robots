from setuptools import find_packages, setup

package_name = 'python_controllers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='anton',
    maintainer_email='a.bredenbeck@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_pos_traj = python_controllers.example_pos_traj:main',
            'example_vel_traj = python_controllers.example_vel_traj:main',
            'jacobian_vel = python_controllers.jacobian_vel:main',
            'pick_place = python_controllers.pick_place:main',
            'competition = python_controllers.competition:main',
            'ik_draw_shape = python_controllers.ik_draw_shape:main',
            'ik_configurations = python_controllers.ik_configurations:main',
            'ik_mult_configs = python_controllers.ik_mult_configs:main',
        ],
    },
)
