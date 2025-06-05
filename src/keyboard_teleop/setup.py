from setuptools import find_packages, setup

package_name = 'keyboard_teleop'

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
    maintainer='dung',
    maintainer_email='dung@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_keyboard = keyboard_teleop.Control_Keyboard:main',
            'simple_parameter = dung_py_ex.parameter_py:main',
            'simple_tf_kinematic = dung_py_ex.simple_tf_kinematic:main',
        ],
    },
)
