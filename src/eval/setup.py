from setuptools import find_packages, setup

package_name = 'eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/exploration_time_evaluation.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='Performance evaluation utilities for swarm exploration runs.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compare_exploration_time = eval.compare_exploration_time:main',
            'exploration_time_evaluator = eval.exploration_time_evaluator:main',
            'performance_evaluator = eval.performance_evaluator_node:main',
            'select_exploration_time_results = eval.select_exploration_time_results:main',
        ],
    },
)
