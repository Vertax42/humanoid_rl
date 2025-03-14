#!/usr/bin/env python3
import rosbag
from sensor_msgs.msg import JointState

def filter_jointstate_messages(input_bag_path, output_bag_path):
    with rosbag.Bag(input_bag_path, 'r') as in_bag, \
         rosbag.Bag(output_bag_path, 'w') as out_bag:

        for topic, msg, timestamp in in_bag.read_messages():
            if msg._type == 'sensor_msgs/JointState':
                # 创建新消息对象，保留原始时间戳和头信息
                filtered_msg = JointState()
                filtered_msg.header = msg.header  # 保留原始时间戳[1](@ref)
                
                # 遍历所有关节名称，筛选非手部关节
                for idx, name in enumerate(msg.name):
                    if "hand" not in name:  # 排除名称含"hand"的关节
                        filtered_msg.name.append(name)
                        
                        # 同步处理 position/velocity/effort 数据
                        if msg.position:  # 检查字段是否存在
                            filtered_msg.position.append(msg.position[idx])
                        if msg.velocity:
                            filtered_msg.velocity.append(msg.velocity[idx])
                        if msg.effort:
                            filtered_msg.effort.append(msg.effort[idx])
                
                # 写入处理后的消息到新bag文件
                out_bag.write(topic, filtered_msg, timestamp)

if __name__ == "__main__":
    input_bag = "../bag/left_bye.bag"  # 替换为实际文件名
    output_bag = "../bag/left_bye_filtered.bag"
    filter_jointstate_messages(input_bag, output_bag)
    print(f"手部关节数据已删除，结果保存至 {output_bag}")