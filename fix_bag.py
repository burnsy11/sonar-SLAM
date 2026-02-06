import sys

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

from dvl_msgs.msg import DVL
from geometry_msgs.msg import TwistStamped

def convert_bag(input_bag_path, output_bag_path, dvl_topic_new='/dvl/data', sonar_topic_new='/sonar/ping'):
    # --- CONFIGURATION ---
    sonar_topic_old = '/oceansim/robot/sonar_ping'
    sonar_type_new = 'oculus_interfaces/msg/Ping'

    dvl_topic = '/dvl/velocity'
    dvl_type_old = 'geometry_msgs/msg/TwistStamped'
    dvl_type_new = 'dvl_msgs/msg/DVL'
    # ---------------------

    # 1. Setup Reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    # 2. Setup Writer
    writer = rosbag2_py.SequentialWriter()
    write_storage_options = rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3')
    writer.open(write_storage_options, converter_options)

    # 3. Create Topics in Output Bag
    topics = reader.get_all_topics_and_types()
    for topic_metadata in topics:
        # Check if this is the topic we want to modify
        if topic_metadata.name == sonar_topic_old:
            print(f"Found target topic: {topic_metadata.name} (Type: {topic_metadata.type})")
            print(f" -> Renaming to: {sonar_topic_new}")
            print(f" -> Changing type to: {sonar_type_new}")

            # Update the metadata object
            topic_metadata.name = sonar_topic_new
            topic_metadata.type = sonar_type_new
        elif topic_metadata.name == dvl_topic:
            print(f"Found target topic: {topic_metadata.name} (Type: {topic_metadata.type})")
            print(f" -> Converting type: {dvl_type_old} -> {dvl_type_new}")
            print(f" -> Renaming to: {dvl_topic_new}")

            # Update the metadata object
            topic_metadata.name = dvl_topic_new
            topic_metadata.type = dvl_type_new

        writer.create_topic(topic_metadata)

    # 4. Copy Messages
    print(f"Converting bag '{input_bag_path}' to '{output_bag_path}'...")
    count = 0
    converted = 0
    sonar_renamed = 0
    dvl_renamed = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()

        # If the message belongs to the sonar topic, redirect it
        if topic == sonar_topic_old:
            topic = sonar_topic_new
            sonar_renamed += 1

        # If the message belongs to the DVL topic, convert its type and rename topic
        if topic == dvl_topic:
            twist_msg = deserialize_message(data, TwistStamped)
            dvl_msg = DVL()
            dvl_msg.header = twist_msg.header
            dvl_msg.time = 0.0
            dvl_msg.velocity = twist_msg.twist.linear
            dvl_msg.fom = 0.0
            dvl_msg.covariance = []
            dvl_msg.altitude = 0.0
            dvl_msg.beams = []
            dvl_msg.velocity_valid = True
            dvl_msg.status = 0
            dvl_msg.time_of_validity = 0
            dvl_msg.time_of_transmission = 0
            dvl_msg.form = ""

            data = serialize_message(dvl_msg)
            topic = dvl_topic_new
            converted += 1
            dvl_renamed += 1

        writer.write(topic, data, t)
        count += 1

    print(
        f"Done! Processed {count} messages. Renamed {sonar_renamed} sonar messages. "
        f"Renamed {dvl_renamed} DVL topics and converted {converted} DVL messages."
    )

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Fix topics and types in a ros2 bag (sqlite3 storage).")
    parser.add_argument("input", help="Input bag directory")
    parser.add_argument("output", help="Output bag directory")
    parser.add_argument("--dvl-new-topic", "-d", default="/dvl/data", help="New topic name for DVL messages (default: /dvl/data)")
    parser.add_argument("--sonar-new-topic", "-s", default="/sonar/ping", help="New topic name for sonar messages (default: /sonar/ping)")

    args = parser.parse_args()

    convert_bag(args.input, args.output, dvl_topic_new=args.dvl_new_topic, sonar_topic_new=args.sonar_new_topic)