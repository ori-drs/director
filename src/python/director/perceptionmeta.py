import abc


class SourceMeta(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def __init__(self, initialisation_object):
        """
        Initialise and start the source
        :param initialisation_object:
        """
        self.consumer = None
        self.init_object = initialisation_object

    @abc.abstractmethod
    def start(self):
        """
        Start the provider
        :return:
        """
        pass

    @abc.abstractmethod
    def stop(self):
        """
        Stop the provider
        :return:
        """
        pass

    def restart(self):
        """
        Restarts this provider
        :return:
        """
        self.stop()
        self.start()

    @abc.abstractmethod
    def set_consumer(self, consumer):
        """
        Set the consumer for this provider
        :param consumer: The consumer which uses this provider to provide it with data. This is a vis.PolyDataItem
        TODO use typing library to enforce this requirement
        :return:
        """
        pass

    @abc.abstractmethod
    def _on_property_changed(self, property_set, property_name):
        """
        This function can be used to pass information about changing properties on the object which uses this as a
        provider. This function should be called from that class' _onPropertyChanged function.
        :param property_set: The set of properties on the object
        :param property_name: The name of the property that changed
        :return:
        """
        pass


class CloudSourceMeta(SourceMeta):

    @abc.abstractmethod
    def get_point_cloud(self, poly_data, only_new_data=True):
        """
        Get a point cloud by combining data from the depth and rgb images
        :param poly_data: This vtk.vtkPolyData item should be populated as the output of this function
        :param only_new_data: If true, only return data if it is different from the value in the previous call to this
        function
        :return:
        """
        pass
    
    @abc.abstractmethod
    def get_sec(self):
        """
        :return: The second at which the last cloud was retrieved
        """
        pass

    @abc.abstractmethod
    def get_nsec(self):
        """
        :return: The nsec at which the last cloud was received
        """
        pass

    @abc.abstractmethod
    def set_fixed_frame(self, fixed_frame):
        """
        Set the fixed frame of the provider
        :param fixed_frame: The name of the fixed frame to use
        :return:
        """
        pass

    @abc.abstractmethod
    def reset_time(self):
        """
        Reset the time used by whatever provides the frame transformations
        :return:
        """
        pass


class ImageSourceMeta(SourceMeta):

    @staticmethod
    @abc.abstractmethod
    def initialise_from_name(name, robot_name):
        """
        Initialise an image source object from a camera name. This is used to help with deferred intialisation
        :param name: The name from which to initialise. This is assumed to refer to a camera name in the director
        configuration
        :param robot_name: The robot this camera is associated with
        :return:
        """
        pass

    @abc.abstractmethod
    def get_current_image_time(self):
        """
        Get the time at which the current image was received
        :return: 0 if no data, otherwise the time of the image as an unsigned long long, computed as
        sec*1000000000 + nsec
        """
        pass

    @abc.abstractmethod
    def get_image(self, poly_data):
        """
        Get the latest image received by the image provider
        :param poly_data: This vtk.vtkPolyData item should be populated as the output of this function
        :return:
        """
        pass

    @abc.abstractmethod
    def compute_texture_coords(self, camera_name, poly_data):
        """

        :param camera_name: Name of the camera to associate with these texture coordinates
        :param poly_data: The geometry to compute coordinates for as a vtk.vtkPolyData object. This will be populated
        with additional fields "tcoords_U" and "tcoords_V" containing the texture coordinates
        :return:
        """

    @abc.abstractmethod
    def get_body_to_camera_transform(self, vtk_transform):
        """
        Get a transform between the camera frame and the robot base frame
        :param vtk_transform: This transform will be populated with the transform
        :return:
        """
        pass

    @abc.abstractmethod
    def reset_time(self):
        """
        Reset the time used by whatever provides the frame transformations
        :return:
        """
        pass


class PointCloudSourceMeta(CloudSourceMeta):

    @abc.abstractmethod
    def set_num_pointclouds(self, num_clouds):
        """
        Set the number of recent pointclouds to keep
        :param num_clouds: Number of clouds to keep
        :return:
        """


class DepthImageSourceMeta(CloudSourceMeta):

    @abc.abstractmethod
    def set_decimate(self, decimate):
        """
        Set the decimation parameter used to get the pointcloud
        :param decimate: Integer representing how much to scale down the pointcloud. The width and height of the cloud
        are divided by this value to get the new size of the cloud after decimation
        :return:
        """
        pass

    @abc.abstractmethod
    def set_remove_size(self, remove_size):
        """
        Set the lower limit on size of components in the received images which should be removed before construction of
        the pointcloud
        :param remove_size: Size in pixels below which to remove components
        :return:
        """
        pass

    @abc.abstractmethod
    def set_range_threshold(self, range_threshold):
        """
        Set the range threshold on the z axis above which points will be filtered out
        :param range_threshold: float threshold
        :return:
        """
        pass


class RosGridMapMeta(SourceMeta):

    @abc.abstractmethod
    def get_mesh(self, poly_data, only_new_data=True):
        """
        Get a mesh representation of the pointcloud from the provider
        :param poly_data: This should be of type vtk.vtkPolyData. The function should populate this with the mesh.
        # TODO update this to use the typing library
        :param only_new_data: If true, only return if the data that would be returned from this call is different to the
        previous one
        :return:
        """
        pass

    @abc.abstractmethod
    def get_point_cloud(self, poly_data, only_new_data=True):
        """
        Get a point cloud from the provider
        :param poly_data: This should be of type vtk.vtkPolyData. The function should populate this with the mesh.
        # TODO update this to use the typing library
        :param only_new_data: If true, only return if the data that would be returned from this call is different to the
        previous one
        :return:
        """
        pass

    @abc.abstractmethod
    def set_color_layer(self, layer_name):
        """
        Defines how colouring of the map should be done based on its available colour layers
        :param layer_name: The name of the layer to use to colour the map
        :return:
        """
        pass

    @abc.abstractmethod
    def set_fixed_frame(self, fixed_frame):
        """
        Set the fixed frame of the provider
        :param fixed_frame: The name of the fixed frame to use
        :return:
        """
        pass

    @abc.abstractmethod
    def reset_time(self):
        """
        Reset the time used by whatever provides the frame transformations
        :return:
        """
        pass


class MarkerSourceMeta(SourceMeta):

    @abc.abstractmethod
    def get_mesh(self, poly_data):
        """
        Get the mesh representation of this marker
        :param poly_data: This should be of type vtk.vtkPolyData. The function should populate this with the mesh.
        # TODO update this to use the typing library
        :return:
        """
        pass

    @abc.abstractmethod
    def set_fixed_frame(self, fixed_frame):
        """
        Set the fixed frame of the provider
        :param fixed_frame: The name of the fixed frame to use
        :return:
        """
        pass


class MarkerArraySourceMeta(SourceMeta):

    @abc.abstractmethod
    def get_mesh(self, poly_data, index=None):
        """
        Get a mesh representation of all markers in the markerarray
        :param poly_data: This should be of type vtk.vtkPolyData. The function should populate this with the meshes.
        # TODO update this to use the typing library
        :param index: The index of the mesh to retrieve
        :return:
        """
        pass

    @abc.abstractmethod
    def get_number_of_mesh(self):
        """

        :return: The number of markers in the array
        """
        pass

    @abc.abstractmethod
    def set_fixed_frame(self, fixed_frame):
        """
        Set the fixed frame of the provider
        :param fixed_frame: The name of the fixed frame to use
        :return:
        """
        pass
