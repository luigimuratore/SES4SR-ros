from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


class Rosbag2Reader:
    """
    A class to easily iterate over the messages in a ROS 2 bag

    Example usage:
    >>> from rosbag2_reader_py import Rosbag2Reader
    >>> from nav_msgs.msg import Odometry
    >>> bag = Rosbag2Reader("path/to/rosbag/file.bag")
    >>> for topic, msg, t in bag:
    ...     print(f"Topic: {topic}")
    ...     if type(msg) is Odometry:
    ...         print(f"Position: {msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}")
    ...     print(f"Message recorded at: {t} ns")
    """

    def __init__(self, path, topics_filter=[], storage_id="sqlite3", serialization_format="cdr"):
        """
        :param path: The path to the rosbag file
        :type path: str
        :param topics_filter: A list of topics names to filter the messages by
        :type topics_filter: list(str)
        :param storage_id: The storage id to use for the rosbag file. Default is "sqlite3", check the metadata.yaml to get this information.
        :type storage_id: str
        :param serialization_format: The serialization format to use for the rosbag file. Default is "cdr", check the metadata.yaml to get this information.
        :type serialization_format: str

        :raises KeyError: If a topic in the topics_filter list is not found in the rosbag file
        """
        self.__path = path
        self.__set_rosbag_options(storage_id, serialization_format)
        self.__reader = rosbag2_py.SequentialReader()
        self.__reader.open(self.__storage_options, self.__converter_options)

        topic_types = self.__reader.get_all_topics_and_types()
        self.__type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        self.set_filter(topics_filter)

    def __iter__(self):
        """Builtin method to use the class as an iterator. Resets the bag reader to the beginning of the file when finished"""
        self.__reset_bag_reader()
        return self

    def __next__(self):
        """
        :return:  A tuple containing the topic name, message object and the timestamp at which the message was recorded
        :rtype: tuple(str, any, int)

        :raises StopIteration: If there are no more messages in the rosbag file
        """
        if self.__reader.has_next():
            (topic, data, t) = self.__reader.read_next()
            msg_type = get_message(self.__type_map[topic])
            msg = deserialize_message(data, msg_type)
            return (topic, msg, t)
        else:
            raise StopIteration

    @property
    def path(self):
        """
        :return: The path to the rosbag file
        :rtype: str
        """
        return self.__path

    @property
    def all_topics(self):
        """
        :return: A dictionary of topics names and their types
        :rtype: dict{str: str}
        """
        return self.__type_map

    @property
    def selected_topics(self):
        """
        :return: A dictionary of topics names and their types that are being used
        :rtype: dict{str: str}
        """
        if self.__storage_filter is None:
            return self.all_topics
        else:
            return self.__selected_topics            

    def set_filter(self, topics):
        """Set a filter to only use the messages from the topics in the topics list"""
        if topics:
            try:
                self.__selected_topics = {topic: self.__type_map[topic] for topic in topics}
            except KeyError as e:
                raise KeyError(f"Could not find topic {e} in the rosbag file")
            self.__storage_filter = rosbag2_py.StorageFilter(topics=topics)
        else:
            self.__storage_filter = None

        self.__reset_bag_reader()

    def reset_filter(self):
        """Reset the filter to use all the topics in the rosbag file"""
        self.__storage_filter = None
        self.__reader.reset_filter()
        self.__reset_bag_reader()

    def __set_rosbag_options(self, storage_id, serialization_format):
        self.__storage_options = rosbag2_py.StorageOptions(uri=self.__path, storage_id=storage_id)

        self.__converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format=serialization_format,
            output_serialization_format=serialization_format,
        )

    def __reset_bag_reader(self):
        """Reset the bag reader to the beginning of the file for usege as an iterator"""
        self.__reader.open(self.__storage_options, self.__converter_options)
        if self.__storage_filter is not None:
            self.__reader.set_filter(self.__storage_filter)
