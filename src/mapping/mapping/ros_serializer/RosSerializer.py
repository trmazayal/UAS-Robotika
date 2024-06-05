from std_msgs.msg import Float32MultiArray

class RosSerializers:
    def __init__(self):
        pass

    def serialize(self, data):
        multi_array = Float32MultiArray()
        multi_array.data = []
        for idx in data:
            multi_array.data.append(idx[0])
            multi_array.data.append(idx[1])
        return multi_array