from target_tf_pkg.fire_assist import fire_value


def test_detected_turns_fire_on():
    assert fire_value(True) == 1


def test_not_detected_turns_fire_off():
    assert fire_value(False) == 0
