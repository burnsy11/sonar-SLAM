from setuptools import setup

package_name = 'pile_inspection_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/launch',
            [
                'launch/pile_inspection_controller.launch.py',
                'launch/pile_inspection_controller_test.launch.py',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Pose and depth controller for pile inspection scan patterns.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pile_inspection_controller_node = pile_inspection_controller.controller_node:main',
            'dummy_tf_sim_node = pile_inspection_controller.dummy_tf_sim_node:main',
        ],
    },
)
