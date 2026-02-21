from setuptools import find_packages, setup

package_name = 'autonomous_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prohype',
    maintainer_email='adhree@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                'navigator = autonomous_navigator.navigator:main',
        ],
    },
)
