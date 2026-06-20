import sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_hover.py <path_to_bag>")
        return

    bag_path = sys.argv[1]
    
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_metadata.name: topic_metadata.type for topic_metadata in topic_types}

    if '/fmu/out/vehicle_odometry' not in type_map:
        print("No /fmu/out/vehicle_odometry topic found in bag!")
        return
        
    msg_type = get_message(type_map['/fmu/out/vehicle_odometry'])
    
    positions_x, positions_y, positions_z = [], [], []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/fmu/out/vehicle_odometry':
            msg = deserialize_message(data, msg_type)
            if hasattr(msg, 'position') and not np.isnan(msg.position[0]):
                positions_x.append(msg.position[0])
                positions_y.append(msg.position[1])
                positions_z.append(msg.position[2])

    print(f"Loaded {len(positions_x)} odometry messages.")
    if not positions_x:
        return

    std_x = np.std(positions_x)
    std_y = np.std(positions_y)
    std_z = np.std(positions_z)
    std_3d = np.sqrt(std_x**2 + std_y**2 + std_z**2)
    
    print("-" * 40)
    print("HOVER POSITION ERROR ANALYSIS (Standard Deviation)")
    print("-" * 40)
    print(f"X-axis Std Dev: {std_x:.4f} meters")
    print(f"Y-axis Std Dev: {std_y:.4f} meters")
    print(f"Z-axis Std Dev: {std_z:.4f} meters")
    print("-" * 40)
    print(f"3D Positional Std Dev: {std_3d:.4f} meters")
    print("-" * 40)

if __name__ == "__main__":
    main()
