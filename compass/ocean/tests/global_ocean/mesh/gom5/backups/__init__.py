import cartopy.crs as ccrs
import cartopy.feature as cfeature
import matplotlib.pyplot as plt
import mpas_tools.mesh.creation.mesh_definition_tools as mdt
import numpy as np
from geometric_features import read_feature_collection
from mpas_tools.cime.constants import constants
from mpas_tools.mesh.creation.signed_distance import (
    mask_from_geojson,
    signed_distance_from_geojson,
)
from mpas_tools.viz.colormaps import register_sci_viz_colormaps

from compass.mesh import QuasiUniformSphericalMeshStep


class GoM5BaseMesh(QuasiUniformSphericalMeshStep):
    """
    A step for creating Gulf of Mexico (GoM) 5km  mesh
    """
    def setup(self):
        """
        Add some input files
        """

        inputs = ['coastline_CUSP.geojson',
                  'land_mask_Kamchatka.geojson',
                  'land_mask_Mexico.geojson',
                  'namelist.split_explicit',
                  'region_Arctic_Ocean.geojson',
                  'region_Bering_Sea.geojson',
                  'region_Bering_Sea_reduced.geojson',
                  'region_Central_America.geojson',
                  'region_Gulf_of_Mexico.geojson',
                  'region_Gulf_Stream_extension.geojson']
        for filename in inputs:
            self.add_input_file(filename=filename,
                                package=self.__module__)

        super().setup()

    def build_cell_width_lat_lon(self):
        """
        Create cell width array for this mesh on a regular latitude-longitude
        grid

        Returns
        -------
        cellWidth : numpy.array
            m x n array of cell width in km

        lon : numpy.array
            longitude in degrees (length n and between -180 and 180)

        lat : numpy.array
            longitude in degrees (length m and between -90 and 90)
        """

        #dlon = 0.1
        dlon = 1.0
        dlat = dlon
        earth_radius = constants['SHR_CONST_REARTH']
        #print('\nCreating cellWidth on a lat-lon grid of: {0:.2f} x {0:.2f} '
        #      'degrees'.format(dlon, dlat))
        print('This can be set higher for faster test generation\n')
        nlon = int(360. / dlon) + 1
        nlat = int(180. / dlat) + 1
        lon = np.linspace(-180., 180., nlon)
        lat = np.linspace(-90., 90., nlat)
        km = 1.0e3

        print('plotting ...')
        plt.switch_backend('Agg')
        fig = plt.figure()
        plt.clf()
        fig.set_size_inches(10.0, 14.0)
        register_sci_viz_colormaps()

########################################################################
#
#  Define cell width for low resolution region
#
########################################################################

        # Expand from 1D to 2D. Pick one of these:
        lowRes = 60.0 # in km
        QU1D = lowRes*np.ones(lat.size)
        _, cellWidth = np.meshgrid(lon, QU1D)

        _plot_cartopy(2, 'low res', cellWidth, '3Wbgy5')
        plotFrame = 3

########################################################################
#
#  Define cell width for high resolution region
#
########################################################################

        # global settings for regionally refines mesh
        highRes = 14.0  # [km]

        fileName = 'region_Central_America'
        transitionWidth = 800.0 * km
        transitionOffset = 0.0
        fc = read_feature_collection('{}.geojson'.format(fileName))
        signedDistance = signed_distance_from_geojson(fc, lon, lat,
                                                      earth_radius,
                                                      max_length=0.25)
        mask = 0.5 * (1 + np.tanh((transitionOffset - signedDistance) /
                                  (transitionWidth / 2.)))
        cellWidth = lowRes * mask + cellWidth * (1 - mask)

        fileName = 'region_Gulf_of_Mexico'
        transitionOffset = 600.0 * km
        transitionWidth = 600.0 * km
        fc = read_feature_collection('{}.geojson'.format(fileName))
        signedDistance = signed_distance_from_geojson(fc, lon, lat,
                                                      earth_radius,
                                                      max_length=0.25)
        maskSmooth = 0.5 * (1 + np.tanh((transitionOffset - signedDistance) /
                                        (transitionWidth / 2.)))
        maskSharp = 0.5 * (1 + np.sign(-signedDistance))
        fc = read_feature_collection('land_mask_Mexico.geojson')
        signedDistance = signed_distance_from_geojson(fc, lon, lat,
                                                      earth_radius,
                                                      max_length=0.25)
        landMask = 0.5 * (1 + np.sign(-signedDistance))
        mask = maskSharp * landMask + maskSmooth * (1 - landMask)
        cellWidth = highRes * mask + cellWidth * (1 - mask)
        _plot_cartopy(plotFrame, fileName + ' mask', mask, 'Blues')
        _plot_cartopy(plotFrame + 1, 'cellWidth ', cellWidth, '3Wbgy5')
        plotFrame += 2

        ax = plt.subplot(6, 2, 1)
        ax.grid(True)
        plt.title('Grid cell size [km] versus latitude')
        plt.legend(loc="upper left")

        plt.savefig('mesh_construction.png', dpi=300)

        return cellWidth, lon, lat


def _plot_cartopy(nPlot, varName, var, map_name):
    ax = plt.subplot(6, 2, nPlot, projection=ccrs.PlateCarree())
    ax.set_global()

    im = ax.imshow(var,
                   origin='lower',
                   transform=ccrs.PlateCarree(),
                   extent=[-180, 180, -90, 90], cmap=map_name,
                   zorder=0)
    ax.add_feature(cfeature.LAND, edgecolor='black', zorder=1)
    gl = ax.gridlines(
        crs=ccrs.PlateCarree(),
        draw_labels=True,
        linewidth=1,
        color='gray',
        alpha=0.5,
        linestyle='-', zorder=2)
    ax.coastlines()
    gl.top_labels = False
    gl.bottom_labels = False
    gl.right_labels = False
    gl.left_labels = False
    plt.colorbar(im, shrink=.9)
    plt.title(varName)
