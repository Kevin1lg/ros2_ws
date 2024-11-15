from setuptools import find_packages, setup

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "count_until_server = actions_py.count_until_server:main",
            "count_until_server1 = actions_py.count_until_server1:main",
            "count_until_server2 = actions_py.count_until_server2:main",
            "count_until_server3 = actions_py.count_until_server3:main",
            "count_until_server4 = actions_py.count_until_server4:main",
            "count_until_server5 = actions_py.count_until_server5:main",
            "count_until_client = actions_py.count_until_client:main",
            "count_until_client1 = actions_py.count_until_client1:main",
            "count_until_client2 = actions_py.count_until_client2:main",
            "count_until_client3 = actions_py.count_until_client3:main",
            "count_until_client4 = actions_py.count_until_client4:main"
        ],
    },
)
