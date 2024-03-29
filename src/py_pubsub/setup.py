from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takud',
    maintainer_email='nagwekarajus@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk = py_pubsub.fk:main',
            'ik = py_pubsub.ik:main',
            'pd = py_pubsub.pd:main',
            'vel_conv = py_pubsub.vel_conv:main',
            'pd_vel = py_pubsub.pd_vel:main',
        ],
    },
)
