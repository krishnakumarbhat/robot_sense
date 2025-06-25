from setuptools import setup

package_name = 'sense'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krishna',
    maintainer_email='krishna@email.com',
    description='Read data from Intel RealSense D435i camera',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd435i_reader = sense.d435i_reader:main'
        ],
    },
)

entry_points={
    'console_scripts': [
        'clean_bot_main = main:main'
    ],
}