from setuptools import setup

package_name = 'decision_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aalap',
    maintainer_email='aalapmanoj@email.com',
    description='ROS 2 Decision Node for Emergency Braking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'decision_logic = decision_node.decision_logic:main',
        ],
    },
)
