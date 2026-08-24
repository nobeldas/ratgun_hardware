from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation


def matrix_to_tf(T, parent, child, stamp):

    tf = TransformStamped()

    tf.header.stamp = stamp
    tf.header.frame_id = parent
    tf.child_frame_id = child

    # Translation
    tf.transform.translation.x = float(T[0, 3])
    tf.transform.translation.y = float(T[1, 3])
    tf.transform.translation.z = float(T[2, 3])

    # Rotation matrix -> quaternion
    q = Rotation.from_matrix(T[:3, :3]).as_quat()

    tf.transform.rotation.x = float(q[0])
    tf.transform.rotation.y = float(q[1])
    tf.transform.rotation.z = float(q[2])
    tf.transform.rotation.w = float(q[3])

    return tf