import sys
import rosbag2_py

def convert_bag(input_bag_path, output_bag_path):
    # --- CONFIGURATION ---
    target_topic_old = '/oceansim/robot/sonar_ping'
    target_topic_new = '/sonar/ping'
    target_type_new  = 'oculus_interfaces/msg/Ping'
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
        if topic_metadata.name == target_topic_old:
            print(f"Found target topic: {topic_metadata.name} (Type: {topic_metadata.type})")
            print(f" -> Renaming to: {target_topic_new}")
            print(f" -> Changing type to: {target_type_new}")
            
            # Update the metadata object
            topic_metadata.name = target_topic_new
            topic_metadata.type = target_type_new

        writer.create_topic(topic_metadata)

    # 4. Copy Messages
    print(f"Converting bag '{input_bag_path}' to '{output_bag_path}'...")
    count = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        # If the message belongs to the old topic, redirect it to the new topic name
        if topic == target_topic_old:
            topic = target_topic_new
        
        writer.write(topic, data, t)
        count += 1

    print(f"Done! Processed {count} messages.")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 fix_bag.py <input_bag_path> <output_bag_path>")
    else:
        convert_bag(sys.argv[1], sys.argv[2])