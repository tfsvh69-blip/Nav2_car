from setuptools import setup


package_name = 'rosmaster_vendor'

setup(
    name=package_name,
    version='3.3.9',
    packages=['Rosmaster_Lib'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (
            'share/' + package_name,
            ['package.xml', 'README.md', 'README.vendor.md'],
        ),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='carcar maintainers',
    maintainer_email='maintainer@example.com',
    description='Vendor packaging for the Yahboom Rosmaster Python driver V3.3.9.',
    license='Proprietary',
)
