from types import SimpleNamespace

import pytest

from auto_aim_tools.analyzer import PreflightAnalyzer, format_report_json


def stamp(sec, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def header(sec, nanosec=0):
    return SimpleNamespace(stamp=stamp(sec, nanosec))


def image(sec=1, width=2, height=2, encoding="bgr8", step=6, data=None):
    if data is None:
        data = bytes(step * height)
    return SimpleNamespace(
        header=header(sec),
        width=width,
        height=height,
        encoding=encoding,
        step=step,
        data=data,
    )


def camera_info(width=2, height=2, model="plumb_bob", d=None, k=None):
    if d is None:
        d = [0.0] * 5
    if k is None:
        k = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
    return SimpleNamespace(
        header=header(1),
        width=width,
        height=height,
        distortion_model=model,
        d=d,
        k=k,
    )


def vision(sec=1, **overrides):
    values = {
        "header": header(sec),
        "yaw": 10.0,
        "yaw_vel": 2.0,
        "yaw_acc": 0.5,
        "pitch": 5.0,
        "pitch_vel": -1.0,
        "pitch_acc": -0.25,
        "roll": 0.0,
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "shoot_speed": 20.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def findings(report, check):
    return [item for item in report["findings"] if item["check"] == check]


def status_for(report, check):
    matches = findings(report, check)
    assert matches, f"missing finding: {check}"
    return matches[0]["status"]


def observe_normal(analyzer):
    for index in range(2):
        now = 0.1 + index * 0.1
        analyzer.observe_image(image(sec=index + 1), now)
        analyzer.observe_camera_info(camera_info(), now)
        analyzer.observe_vision(vision(sec=index + 1), now)


def test_normal_messages_pass_with_explicit_profile_and_clock_domain():
    analyzer = PreflightAnalyzer(
        timeout_s=1.0,
        vehicle_profile="new_turtle",
        shared_clock_domain=True,
        start_s=0.0,
    )
    observe_normal(analyzer)

    report = analyzer.build_report(now_s=0.25)

    assert report["overall"] == "PASS"
    assert status_for(report, "image.data_length") == "PASS"
    assert status_for(report, "camera_info.K") == "PASS"
    assert status_for(report, "vision.acceleration_finite") == "PASS"
    assert status_for(report, "vision.quaternion_format") == "PASS"
    assert status_for(report, "image_vision.timestamp_delta") == "PASS"
    scalars = findings(report, "vision.scalar_finite")[0]["details"]
    assert scalars["units"]["yaw_vel"] == "degree/s"
    assert scalars["values"]["roll"] == 0.0
    acceleration = findings(report, "vision.acceleration_finite")[0]["details"]
    assert acceleration["units"]["yaw_acc"] == "degree/s²"
    assert acceleration["values"]["pitch_acc"] == -0.25


def test_missing_topic_and_timeout_are_failures():
    analyzer = PreflightAnalyzer(timeout_s=0.2, start_s=0.0)
    analyzer.observe_image(image(), 0.1)
    analyzer.observe_camera_info(camera_info(), 0.1)

    report = analyzer.build_report(now_s=0.5)

    missing = [
        item for item in findings(report, "topic.received")
        if item["topic"] == "/Vision_data"
    ]
    image_timeout = [
        item for item in findings(report, "topic.timeout")
        if item["topic"] == "/image_raw"
    ]
    assert missing[0]["status"] == "FAIL"
    assert image_timeout[0]["status"] == "FAIL"


def test_empty_image_and_bad_step_are_reported_without_exception():
    analyzer = PreflightAnalyzer(start_s=0.0)
    analyzer.observe_image(image(width=0, height=0, step=0, data=b""), 0.1)

    report = analyzer.build_report(now_s=0.2)

    assert status_for(report, "image.dimensions") == "FAIL"
    assert status_for(report, "image.step") == "FAIL"
    assert status_for(report, "image.data_length") == "FAIL"


@pytest.mark.parametrize(
    "message,check",
    [
        (camera_info(k=[float("nan")] * 9), "camera_info.K"),
        (camera_info(d=[0.0] * 4), "camera_info.D"),
        (camera_info(width=0), "camera_info.dimensions"),
    ],
)
def test_invalid_camera_info_is_a_failure(message, check):
    analyzer = PreflightAnalyzer(start_s=0.0)
    analyzer.observe_camera_info(message, 0.1)
    assert status_for(analyzer.build_report(0.2), check) == "FAIL"


def test_timestamp_rollback_is_retained_after_a_later_good_stamp():
    analyzer = PreflightAnalyzer(start_s=0.0)
    analyzer.observe_image(image(sec=10), 0.1)
    analyzer.observe_image(image(sec=9), 0.2)
    analyzer.observe_image(image(sec=11), 0.3)

    report = analyzer.build_report(now_s=0.4)
    match = [
        item for item in findings(report, "header.timestamp_monotonic")
        if item["topic"] == "/image_raw"
    ][0]
    assert match["status"] == "FAIL"
    assert match["details"]["rollbacks"] == 1


@pytest.mark.parametrize(
    "overrides,check",
    [
        ({"yaw": 180.1}, "vision.yaw_range"),
        ({"yaw_vel": float("inf")}, "vision.scalar_finite"),
        ({"yaw_acc": float("nan")}, "vision.acceleration_finite"),
        ({"quaternion": [1.0, 0.0, 0.0]}, "vision.quaternion_format"),
        ({"quaternion": [1.0, 0.0, 0.0, float("inf")]}, "vision.quaternion_format"),
    ],
)
def test_invalid_vision_fields_are_failures(overrides, check):
    analyzer = PreflightAnalyzer(vehicle_profile="new_turtle", start_s=0.0)
    analyzer.observe_vision(vision(**overrides), 0.1)
    assert status_for(analyzer.build_report(0.2), check) == "FAIL"


def test_pitch_is_undetermined_without_explicit_profile():
    analyzer = PreflightAnalyzer(vehicle_profile="unselected", start_s=0.0)
    analyzer.observe_vision(vision(pitch=999.0), 0.1)

    match = findings(analyzer.build_report(0.2), "vision.pitch_profile")[0]
    assert match["status"] == "WARN"
    assert "无法判定" in match["reason"]


def test_pitch_profile_range_is_applied_only_when_selected():
    analyzer = PreflightAnalyzer(vehicle_profile="dog_leg", start_s=0.0)
    analyzer.observe_vision(vision(pitch=31.1), 0.1)
    assert status_for(analyzer.build_report(0.2), "vision.pitch_profile") == "FAIL"


def test_missing_acceleration_fields_are_explicitly_unavailable():
    message = vision()
    del message.yaw_acc
    del message.pitch_acc
    analyzer = PreflightAnalyzer(start_s=0.0)
    analyzer.observe_vision(message, 0.1)

    match = findings(analyzer.build_report(0.2), "vision.acceleration_finite")[0]
    assert match["status"] == "WARN"
    assert match["details"]["missing_fields"] == ["yaw_acc", "pitch_acc"]


def test_timestamps_are_not_compared_without_explicit_clock_declaration():
    analyzer = PreflightAnalyzer(shared_clock_domain=False, start_s=0.0)
    analyzer.observe_image(image(sec=100), 0.1)
    analyzer.observe_vision(vision(sec=1), 0.1)

    report = analyzer.build_report(0.2)
    match = findings(report, "image_vision.clock_domain")[0]
    assert match["status"] == "WARN"
    assert match["details"]["compared"] is False
    assert "时间基准未确认" in match["reason"]
    assert not findings(report, "image_vision.timestamp_delta")


def test_json_report_keeps_all_three_status_names():
    analyzer = PreflightAnalyzer(start_s=0.0)
    analyzer.observe_image(image(), 0.1)
    rendered = format_report_json(analyzer.build_report(0.2))
    assert '"PASS"' in rendered
    assert '"WARN"' in rendered
    assert '"FAIL"' in rendered
