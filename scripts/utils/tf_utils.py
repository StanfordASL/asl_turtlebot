from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

from tf_conversions import transformations

quaternion2euler = transformations.euler_from_quaternion
euler2quaternion = transformations.quaternion_from_euler

def create_transform_msg(frm, to, translation, rotation, time=None):
    """Helper function for making TransformStamped messages for all transforms.

    Helper function for making TransformStamped messages for all transforms.
    Both static and dynamic (normal) transforms need a TransformStamped messages
    because the Transform message does not have from and to frame id values. For
    static transforms, the time field does not matter.

    Args:
        frm (string): which frame is this transform converting the pose from
        to (string): which frame is this transform converting the pose to
        translation (list(float)): the translation from the `frm` frame to `to`
        rotation (list(float)): the rotation from the `frm` frame to `to`

        time (Optional[float]): the time stamp of the message

    Returns:
        TransformStamped: stamped transform message
    """

    time = time if time is not None else rospy.Time.now()
    header = Header(stamp=time, frame_id=frm)
    assert len(rotation) == 3 or len(rotation) == 4
    rotation = rotation if len(rotation) == 4 else euler2quaternion(*rotation)
    transform = Transform(
        translation=Vector3(*translation), rotation=Quaternion(*rotation)
    )
    return TransformStamped(
        header=header, child_frame_id=to, transform=transform
    )
