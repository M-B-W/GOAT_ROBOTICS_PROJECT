from setuptools import setup

package_name = 'butler_bot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={'butler_bot': ['resource/*']},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vishal',
    maintainer_email='vishal@example.com',
    description='Task execution for Butler Bot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'butler_task1_node = butler_bot.butler_task1_node:main',
            'butler_task2_node = butler_bot.butler_task2_node:main',
            'butler_task3_node = butler_bot.butler_task3_node:main',
            'butler_task4_node = butler_bot.butler_task4_node:main',
            'butler_task5_node = butler_bot.butler_task5_node:main',
            'butler_task6_node = butler_bot.butler_task6_node:main',
            'butler_task7_node = butler_bot.butler_task7_node:main',

            'butler_gui = butler_bot.butler_task_gui:main',

        ],
    },
)
