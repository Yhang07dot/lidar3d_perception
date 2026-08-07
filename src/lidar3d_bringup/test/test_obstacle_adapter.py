from geometry_msgs.msg import TransformStamped

from lidar3d_bringup.obstacle_adapter import ObstacleAdapter


def test_slope_metadata_is_rewritten_in_target_frame():
    transform = TransformStamped()
    transform.transform.translation.x = 0.5
    transform.transform.translation.y = -0.2
    transform.transform.translation.z = 1.5
    transform.transform.rotation.w = 1.0

    adapter = object.__new__(ObstacleAdapter)
    text = (
        'passable_slope apex_x=8.00 apex_y=1.25 apex_z=-0.30 '
        'span_x=5.50 grade_deg=4.20 cells=18 c=1.00'
    )

    converted = adapter._transform_slope_metadata(text, transform)

    assert converted == (
        'passable_slope apex_x=8.50 apex_y=1.05 apex_z=1.20 '
        'span_x=5.50 grade_deg=4.20 cells=18 c=1.00'
    )
