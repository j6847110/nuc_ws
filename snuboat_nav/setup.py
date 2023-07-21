from setuptools import find_packages
from setuptools import setup

package_name = 'snuboat_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiyoung',
    maintainer_email='mouse890@snu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		# 'WP_Following = snuboat_navigation.WP_Following:main',
        'gnss = snuboat_nav.gnss_converter:main',
        'lidar = snuboat_nav.lidar_converter:main',
        'dummy_enu_pub=snuboat_nav.dummy_enu_publisher:main',
        'dummy_gps_pub=snuboat_nav.dummy_gps_publisher:main',
        'obs = snuboat_nav.obstacle_avoidance:main',
        'obs_rev=snuboat_nav.obstacle_avoidance_rev:main',
        'pwm_cvt = snuboat_nav.pwm_converter:main',
        'diff = snuboat_nav.differentiater:main',
        'vis = snuboat_nav.visualizer_z:main',
        'vis_lf = snuboat_nav.visualizer_lf:main',
        ],
    },
)
