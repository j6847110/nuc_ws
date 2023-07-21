from setuptools import setup

package_name = 'snuboat_nav90'

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
    maintainer='snuboat',
    maintainer_email='j6847110@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obs = snuboat_nav90.obstacle_avoidance:main',
            'pwm_cvt = snuboat_nav90.pwm_converter:main',
        ],
    },
)
