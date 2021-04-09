from compass.config import add_config
from compass.testgroup import TestGroup
from compass.ocean.tests.ice_shelf_2d.default import Default
from compass.ocean.tests.ice_shelf_2d.restart_test import RestartTest


class IceShelf2d(TestGroup):
    """
    A test group for ice-shelf 2D test cases
    """
    def __init__(self, mpas_core):
        """
        mpas_core : compass.MpasCore
            the MPAS core that this test group belongs to
        """
        super().__init__(mpas_core=mpas_core, name='ice_shelf_2d')

        for resolution in ['5km']:
            Default(test_group=self, resolution=resolution)
            RestartTest(test_group=self, resolution=resolution)


def configure(name, resolution, config):
    """
    Modify the configuration options for this test case

    Parameters
    ----------
    name : str
        the name of the test case

    resolution : str
        The resolution of the test case

    config : configparser.ConfigParser
        Configuration options for this test case
    """
    res_params = {'5km': {'nx': 10, 'ny': 44, 'dc': 5e3}}

    if resolution not in res_params:
        raise ValueError('Unsupported resolution {}. Supported values are: '
                         '{}'.format(resolution, list(res_params)))
    res_params = res_params[resolution]
    for param in res_params:
        config.set('ice_shelf_2d', param, '{}'.format(res_params[param]))

    add_config(config, 'compass.ocean.tests.ice_shelf_2d.{}'.format(name),
               '{}.cfg'.format(name), exception=False)
