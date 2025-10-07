import struct
import time
import math

from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Quaternion

def bytes_to_twist(data: bytes) -> Twist:
    # 6 doubles, little-endian
    values = [struct.unpack('<d', data[i:i+8])[0] for i in range(0, 48, 8)]
    return Twist(
        linear=Vector3(x=values[0], y=values[1], z=values[2]),
        angular=Vector3(x=values[3], y=values[4], z=values[5])
    )

def bytes_to_pose_stamped(data: bytes, pose_offset: int = 16) -> PoseStamped:
    if len(data) < pose_offset + 56:
        raise ValueError(f"Expected at least {pose_offset + 56} bytes for PoseStamped, got {len(data)}")
    # Read 7 doubles after the offset
    values = [struct.unpack('<d', data[pose_offset + i : pose_offset + i + 8])[0] for i in range(0, 56, 8)]

    pose_msg = PoseStamped()
    # Stamp with current wall-clock time
    t = time.time()
    pose_msg.header.stamp.sec = int(t)
    pose_msg.header.stamp.nanosec = int((t % 1) * 1e9)
    pose_msg.header.frame_id = "base_link"

    pose_msg.pose.position = Point(x=values[0], y=values[1], z=values[2])
    pose_msg.pose.orientation = Quaternion(x=values[3], y=values[4], z=values[5], w=values[6])
    return pose_msg


def bytes_to_ovr2ros_inputs(data: bytes) -> OVR2ROSInputs:

    if len(data) < 18:
        raise ValueError(f"Expected 18 bytes, got {len(data)}")

    # 2 x bool (<??) + 4 x float32 (<ffff)
    button_upper, button_lower, x, y, index, middle = struct.unpack('<??ffff', data[0:18])

    msg = OVR2ROSInputs()
    msg.button_upper = button_upper
    msg.button_lower = button_lower
    msg.thumb_stick_horizontal = x
    msg.thumb_stick_vertical = y
    msg.press_index = index
    msg.press_middle = middle

    return msg

def bytes_to_ovr2ros_haptic_feedback(data: bytes) -> OVR2ROSHapticFeedback:
    freq, amp = struct.unpack('<dd', data[0:16])
    msg = OVR2ROSHapticFeedback()
    msg.frequency = freq
    msg.amplitude = amp
    return msg

def convert_data(topic: str, data: bytes):
    if topic in ["q2r_right_hand_twist", "q2r_left_hand_twist", "dice_twist", "q2r_twist"]:
        return bytes_to_twist(data)
    elif topic in ["q2r_right_hand_pose", "q2r_left_hand_pose"]:
        return bytes_to_pose_stamped(data)
    elif topic in ["q2r_right_hand_inputs", "q2r_left_hand_inputs"]:
        return bytes_to_ovr2ros_inputs(data)
    elif topic in ["q2r_right_hand_haptic_feedback", "q2r_left_hand_haptic_feedback"]:
        return bytes_to_ovr2ros_haptic_feedback(data)
    else:
        print(f"[WARNING] Unknown topic '{topic}', cannot convert.")
        return None
