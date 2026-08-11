import random
from pathlib import Path

import pytest
import yaml

from env.utils import populate_scene


EXCLUDED_SCENE_OBJECTS = {"tuna_fish_can", "pudding_box"}


def _objects(*names):
    return [{"name": name} for name in names]


def _names(objects):
    return [obj["name"] for obj in objects]


def _scene_categories():
    return {
        "cylindrical_can": ["tomato_soup_can"],
        "banana": ["banana"],
        "round_top": [
            "apple",
            "lemon",
            "peach",
            "pear",
            "orange",
            "plum",
            "baseball",
            "tennis_ball",
            "racquetball",
        ],
        "box": [
            "gelatin_box",
            "sponge",
            "foam_brick",
            "rubiks_cube",
        ],
        "tool_top": ["hammer"],
    }


def _category_pool_objects():
    categories = _scene_categories()
    return _objects(
        *(name for names in categories.values() for name in names)
    )


def test_fixed_objects_are_kept_first_and_random_slots_are_filled():
    objects = _objects(
        "tomato_soup_can",
        "gelatin_box",
        "banana",
        "hammer",
        "foam_brick",
        "apple",
        "lemon",
    )

    selected = populate_scene.select_scene_objects(
        objects,
        random_object_count=5,
        fixed_object_names=["tomato_soup_can", "banana", "hammer"],
        rng=random.Random(11),
    )
    selected_names = _names(selected)

    assert selected_names[:3] == ["tomato_soup_can", "banana", "hammer"]
    assert len(selected_names) == 5
    assert len(set(selected_names)) == len(selected_names)
    assert set(selected_names[3:]).issubset(
        {"gelatin_box", "foam_brick", "apple", "lemon"}
    )


def test_fixed_objects_are_a_minimum_even_when_count_is_smaller():
    objects = _objects("tomato_soup_can", "banana", "hammer", "apple")

    selected = populate_scene.select_scene_objects(
        objects,
        random_object_count=2,
        fixed_object_names=["tomato_soup_can", "banana", "hammer"],
        rng=random.Random(3),
    )

    assert _names(selected) == ["tomato_soup_can", "banana", "hammer"]


def test_zero_random_count_returns_only_fixed_objects_in_order():
    objects = _objects(
        "tomato_soup_can",
        "sponge",
        "hammer",
        "banana",
        "tennis_ball",
        "baseball",
        "apple",
    )

    selected = populate_scene.select_scene_objects(
        objects,
        random_object_count=0,
        fixed_object_names=[
            "tomato_soup_can",
            "sponge",
            "hammer",
            "banana",
            "tennis_ball",
            "baseball",
        ],
        rng=random.Random(7),
    )

    assert _names(selected) == [
        "tomato_soup_can",
        "sponge",
        "hammer",
        "banana",
        "tennis_ball",
        "baseball",
    ]


def test_assign_placement_slots_sets_fixed_positions_without_mutating_inputs():
    objects = [
        {
            "name": "tomato_soup_can",
            "type": "mesh",
            "position_range": {"x": [-1, 1], "y": [-1, 1], "z": [0, 0.02]},
        },
        {
            "name": "banana",
            "type": "mesh",
            "position_range": {"x": [-1, 1], "y": [-1, 1], "z": [0, 0.02]},
        },
    ]
    slots = [[-0.74, -0.08, 0.0], [-0.55, -0.08, 0.0]]

    assigned = populate_scene.assign_placement_slots(objects, slots)

    assert [obj["position"] for obj in assigned] == slots
    assert all("position_range" not in obj for obj in assigned)
    assert "position_range" in objects[0]
    assert "position" not in objects[0]


def test_assign_placement_slots_rejects_too_few_slots():
    objects = _objects("tomato_soup_can", "banana")

    with pytest.raises(ValueError, match="placement_slots"):
        populate_scene.assign_placement_slots(objects, [[-0.74, -0.08, 0.0]])


def test_default_config_uses_fixed_priority_random_scene_layout():
    config_path = (
        Path(__file__).resolve().parents[1]
        / "env"
        / "config"
        / "base_env.yaml"
    )
    config = yaml.safe_load(config_path.read_text())

    sim_config = config["sim"]
    assert sim_config["camera_names"] == ["camera_orbbec"]
    assert sim_config["camera_size"] == [1280, 720]
    assert sim_config["render_fps"] == 0.2
    assert sim_config["enable_pointcloud_camera1"] is False
    assert sim_config["enable_depth_camera1"] is True
    assert sim_config["enable_pointcloud_camera2"] is False
    assert sim_config["enable_depth_camera2"] is False

    assert config["scene_mode"] == "mix"
    assert config["assigned_object_names"] == []
    assert config["scene_object_categories"] == _scene_categories()
    assert config["fixed_object_names"] == [
        "tomato_soup_can",
        "banana",
        "apple",
        "foam_brick",
        "hammer",
    ]
    assert config["random_object_count"] == 6
    assert config["placement_slots"] == [
        [-0.74, -0.08, 0.0],
        [-0.55, -0.08, 0.0],
        [-0.36, -0.08, 0.0],
        [-0.74, 0.20, 0.0],
        [-0.55, 0.30, 0.0],
        [-0.85, 0.35, 0.0],
    ]

    by_name = {obj["name"]: obj for obj in config["objects"]}
    assert len(by_name) == 16
    assert not EXCLUDED_SCENE_OBJECTS.intersection(by_name)
    configured_category_names = {
        name
        for names in config["scene_object_categories"].values()
        for name in names
    }
    assert not EXCLUDED_SCENE_OBJECTS.intersection(configured_category_names)
    for name in config["fixed_object_names"]:
        assert "position_range" in by_name[name]
        assert "position" not in by_name[name]

    selected = populate_scene.select_scene_objects(
        config["objects"],
        random_object_count=config["random_object_count"],
        fixed_object_names=config["fixed_object_names"],
        rng=random.Random(17),
    )
    selected_names = _names(selected)

    fixed_names = config["fixed_object_names"]
    assert selected_names[:5] == fixed_names
    assert len(selected_names) == 6
    assert len(set(selected_names)) == len(selected_names)
    assert selected_names[5] not in fixed_names
    assigned = populate_scene.assign_placement_slots(
        selected,
        config["placement_slots"],
    )
    assert [obj["position"] for obj in assigned] == config["placement_slots"]
    assert all("position_range" not in obj for obj in assigned)


def test_unknown_fixed_object_name_raises_clear_error():
    objects = _objects("banana", "hammer")

    with pytest.raises(ValueError, match="missing_can"):
        populate_scene.select_scene_objects(
            objects,
            random_object_count=5,
            fixed_object_names=["banana", "missing_can"],
        )


def test_random_mode_selects_one_per_category_then_a_unique_sixth_object():
    categories = _scene_categories()

    selected = populate_scene.select_scene_objects_for_mode(
        _category_pool_objects(),
        scene_mode="random",
        random_object_count=6,
        fixed_object_names=[
            "tomato_soup_can",
            "banana",
            "apple",
            "foam_brick",
            "hammer",
        ],
        scene_object_categories=categories,
        rng=random.Random(11),
    )
    selected_names = _names(selected)

    assert len(selected_names) == 6
    assert len(set(selected_names)) == 6
    for slot_index, category_names in enumerate(categories.values()):
        assert selected_names[slot_index] in category_names
    assert selected_names[1] == "banana"
    assert selected_names[4] == "hammer"
    assert selected_names[5] not in selected_names[:5]


def test_random_mode_remains_valid_across_multiple_seeds():
    categories = _scene_categories()
    objects = _category_pool_objects()

    results = []
    for seed in range(20):
        selected_names = _names(
            populate_scene.select_scene_objects_for_mode(
                objects,
                scene_mode="random",
                random_object_count=6,
                fixed_object_names=[],
                scene_object_categories=categories,
                rng=random.Random(seed),
            )
        )
        assert len(selected_names) == len(set(selected_names)) == 6
        assert not EXCLUDED_SCENE_OBJECTS.intersection(selected_names)
        assert selected_names[1] == "banana"
        assert selected_names[4] == "hammer"
        results.append(tuple(selected_names))

    assert len(set(results)) > 1


def test_mix_mode_dispatch_preserves_legacy_selection_exactly():
    objects = _category_pool_objects()
    fixed_names = [
        "tomato_soup_can",
        "banana",
        "apple",
        "foam_brick",
        "hammer",
    ]

    legacy = populate_scene.select_scene_objects(
        objects,
        random_object_count=6,
        fixed_object_names=fixed_names,
        rng=random.Random(17),
    )
    dispatched = populate_scene.select_scene_objects_for_mode(
        objects,
        scene_mode="mix",
        random_object_count=6,
        fixed_object_names=fixed_names,
        scene_object_categories=_scene_categories(),
        rng=random.Random(17),
    )

    assert _names(dispatched) == _names(legacy)


def test_scene_mode_rejects_unknown_value():
    with pytest.raises(ValueError, match="scene_mode.*mix.*random"):
        populate_scene.select_scene_objects_for_mode(
            _category_pool_objects(),
            scene_mode="surprise",
            random_object_count=6,
            fixed_object_names=[],
            scene_object_categories=_scene_categories(),
        )


def test_random_mode_rejects_empty_required_category():
    categories = _scene_categories()
    categories["round_top"] = []

    with pytest.raises(ValueError, match="round_top.*must not be empty"):
        populate_scene.select_category_scene_objects(
            _category_pool_objects(),
            categories,
        )


def test_random_mode_rejects_unknown_category_object():
    categories = _scene_categories()
    categories["box"].append("missing_box")

    with pytest.raises(ValueError, match="box.*missing_box.*object pool"):
        populate_scene.select_category_scene_objects(
            _category_pool_objects(),
            categories,
        )


def test_random_mode_rejects_duplicate_category_membership():
    categories = _scene_categories()
    categories["box"].append("apple")

    with pytest.raises(ValueError, match="apple.*round_top.*box"):
        populate_scene.select_category_scene_objects(
            _category_pool_objects(),
            categories,
        )


def test_random_mode_rejects_missing_required_category():
    categories = _scene_categories()
    categories.pop("banana")

    with pytest.raises(ValueError, match="missing required.*banana"):
        populate_scene.select_category_scene_objects(
            _category_pool_objects(),
            categories,
        )


def test_random_mode_rejects_no_remaining_sixth_object():
    categories = {
        "cylindrical_can": ["tomato_soup_can"],
        "banana": ["banana"],
        "round_top": ["apple"],
        "box": ["foam_brick"],
        "tool_top": ["hammer"],
    }
    objects = _objects(
        "tomato_soup_can",
        "banana",
        "apple",
        "foam_brick",
        "hammer",
    )

    with pytest.raises(ValueError, match="sixth.*remaining object"):
        populate_scene.select_category_scene_objects(objects, categories)


def test_assign_mode_with_six_objects_preserves_exact_order():
    assigned_names = [
        "hammer",
        "banana",
        "tomato_soup_can",
        "apple",
        "foam_brick",
        "baseball",
    ]

    selected = populate_scene.select_assigned_scene_objects(
        _category_pool_objects(),
        assigned_names,
        rng=random.Random(7),
    )

    assert _names(selected) == assigned_names


@pytest.mark.parametrize("assigned_count", range(1, 6))
def test_assign_mode_keeps_prefix_and_randomly_fills_without_duplicates(
    assigned_count,
):
    assigned_names = [
        "hammer",
        "banana",
        "tomato_soup_can",
        "apple",
        "foam_brick",
    ][:assigned_count]

    selected_names = _names(
        populate_scene.select_assigned_scene_objects(
            _category_pool_objects(),
            assigned_names,
            rng=random.Random(assigned_count),
        )
    )

    assert selected_names[:assigned_count] == assigned_names
    assert len(selected_names) == len(set(selected_names)) == 6
    assert not set(selected_names[assigned_count:]).intersection(assigned_names)
    assert not EXCLUDED_SCENE_OBJECTS.intersection(selected_names)


def test_assign_mode_blank_assignment_randomly_selects_six_unique_objects():
    objects = _category_pool_objects()
    results = []

    for seed in range(20):
        selected_names = _names(
            populate_scene.select_assigned_scene_objects(
                objects,
                [],
                rng=random.Random(seed),
            )
        )
        assert len(selected_names) == len(set(selected_names)) == 6
        assert not EXCLUDED_SCENE_OBJECTS.intersection(selected_names)
        results.append(tuple(selected_names))

    assert len(set(results)) > 1


def test_assign_mode_dispatches_with_assigned_prefix():
    selected_names = _names(
        populate_scene.select_scene_objects_for_mode(
            _category_pool_objects(),
            scene_mode="assign",
            random_object_count=6,
            fixed_object_names=[],
            scene_object_categories=_scene_categories(),
            assigned_object_names=["banana", "hammer"],
            rng=random.Random(9),
        )
    )

    assert selected_names[:2] == ["banana", "hammer"]
    assert len(selected_names) == len(set(selected_names)) == 6


def test_assign_mode_rejects_unknown_object():
    with pytest.raises(ValueError, match="unknown assigned object.*missing"):
        populate_scene.select_assigned_scene_objects(
            _category_pool_objects(),
            ["banana", "missing"],
        )


@pytest.mark.parametrize("excluded_name", sorted(EXCLUDED_SCENE_OBJECTS))
def test_assign_mode_rejects_excluded_scene_object(excluded_name):
    with pytest.raises(ValueError, match="unknown assigned object"):
        populate_scene.select_assigned_scene_objects(
            _category_pool_objects(),
            [excluded_name],
        )


def test_assign_mode_rejects_duplicate_object():
    with pytest.raises(ValueError, match="duplicate assigned object.*banana"):
        populate_scene.select_assigned_scene_objects(
            _category_pool_objects(),
            ["banana", "banana"],
        )


def test_assign_mode_rejects_more_than_six_objects():
    with pytest.raises(ValueError, match="at most 6.*got 7"):
        populate_scene.select_assigned_scene_objects(
            _category_pool_objects(),
            [
                "tomato_soup_can",
                "gelatin_box",
                "banana",
                "apple",
                "lemon",
                "peach",
                "pear",
            ],
        )


def test_assign_mode_rejects_pool_with_fewer_than_six_unique_objects():
    with pytest.raises(ValueError, match="requires at least 6 unique objects"):
        populate_scene.select_assigned_scene_objects(
            _objects("banana", "hammer", "apple", "foam_brick", "baseball"),
            [],
        )
