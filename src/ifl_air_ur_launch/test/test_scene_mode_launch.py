import importlib.util
from pathlib import Path

import pytest


LAUNCH_FILE = (
    Path(__file__).resolve().parents[1]
    / "launch"
    / "cell_small_full_mujoco_moveit.launch.py"
)
SCENE_CONFIG_FILE = (
    Path(__file__).resolve().parents[2]
    / "ifl_air_mujoco_sim"
    / "env"
    / "config"
    / "base_env.yaml"
)
EXPECTED_SCENE_OBJECTS = (
    "tomato_soup_can",
    "gelatin_box",
    "banana",
    "apple",
    "lemon",
    "peach",
    "pear",
    "orange",
    "plum",
    "sponge",
    "hammer",
    "baseball",
    "tennis_ball",
    "racquetball",
    "foam_brick",
    "rubiks_cube",
)


@pytest.fixture(scope="module")
def launch_module():
    spec = importlib.util.spec_from_file_location("full_cell_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("raw_mode", "expected"),
    [
        ("mix", "mix"),
        (" random ", "random"),
        ("MIX", "mix"),
        ("Assign", "assign"),
    ],
)
def test_explicit_scene_mode_is_normalized_without_prompt(
    launch_module,
    raw_mode,
    expected,
):
    def fail_if_prompted(_prompt):
        raise AssertionError("explicit scene mode must not prompt")

    assert launch_module.resolve_scene_mode(
        raw_mode,
        input_fn=fail_if_prompted,
    ) == expected


def test_prompt_accepts_random(launch_module):
    prompts = []

    result = launch_module.resolve_scene_mode(
        "prompt",
        input_fn=lambda prompt: prompts.append(prompt) or " random ",
    )

    assert result == "random"
    assert prompts == ["Select scene mode [mix/random/assign]: "]


def test_prompt_retries_invalid_input_before_accepting_mix(launch_module):
    responses = iter(["", "unknown", "MiX"])
    messages = []

    result = launch_module.resolve_scene_mode(
        "prompt",
        input_fn=lambda _prompt: next(responses),
        output_fn=messages.append,
    )

    assert result == "mix"
    assert len(messages) == 2
    assert all(
        "random" in message and "mix" in message and "assign" in message
        for message in messages
    )


def test_prompt_closed_input_fails_cleanly(launch_module):
    def closed_input(_prompt):
        raise EOFError

    with pytest.raises(RuntimeError, match="scene mode input stream closed"):
        launch_module.resolve_scene_mode("prompt", input_fn=closed_input)


def test_prompt_keyboard_interrupt_is_not_swallowed(launch_module):
    def interrupted_input(_prompt):
        raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        launch_module.resolve_scene_mode("prompt", input_fn=interrupted_input)


def test_invalid_explicit_scene_mode_fails_without_prompt(launch_module):
    with pytest.raises(ValueError, match="scene_mode.*mix.*random"):
        launch_module.resolve_scene_mode("unsupported")


def test_launch_process_forwards_random_as_hydra_override(launch_module):
    class FakeContext:
        @staticmethod
        def perform_substitution(_substitution):
            return "random"

    actions = launch_module.launch_mujoco_ros_interface(FakeContext())

    assert len(actions) == 1
    shell_command = "".join(
        substitution.text for substitution in actions[0].cmd[2]
    )
    assert "scene_mode=random" in shell_command


def test_scene_prompt_action_precedes_all_launch_children(launch_module):
    entities = list(launch_module.generate_launch_description().entities)
    first_child_index = next(
        index
        for index, entity in enumerate(entities)
        if not isinstance(entity, launch_module.DeclareLaunchArgument)
    )

    assert all(
        isinstance(entity, launch_module.DeclareLaunchArgument)
        for entity in entities[:first_child_index]
    )
    assert isinstance(entities[first_child_index], launch_module.OpaqueFunction)


def test_load_available_object_names_uses_yaml_order(launch_module, tmp_path):
    config_path = tmp_path / "base_env.yaml"
    config_path.write_text(
        "objects:\n"
        "  - name: banana\n"
        "  - name: apple\n"
        "  - name: foam_brick\n",
        encoding="utf-8",
    )

    assert launch_module.load_available_object_names(config_path) == (
        "banana",
        "apple",
        "foam_brick",
    )


def test_default_scene_config_exposes_sixteen_objects(launch_module):
    assert launch_module.load_available_object_names(
        SCENE_CONFIG_FILE
    ) == EXPECTED_SCENE_OBJECTS


def test_default_prompt_reports_sixteen_objects(launch_module):
    messages = []

    result = launch_module.resolve_assigned_object_names(
        EXPECTED_SCENE_OBJECTS,
        input_fn=lambda _prompt: "",
        output_fn=messages.append,
    )

    assert result == []
    assert messages == [
        "Available objects (16): " + ", ".join(EXPECTED_SCENE_OBJECTS)
    ]


@pytest.mark.parametrize("excluded_name", ["tuna_fish_can", "pudding_box"])
def test_default_scene_pool_rejects_excluded_objects(
    launch_module,
    excluded_name,
):
    with pytest.raises(ValueError, match=f"unknown object: {excluded_name}"):
        launch_module.parse_assigned_object_names(
            excluded_name,
            EXPECTED_SCENE_OBJECTS,
        )


@pytest.mark.parametrize(
    ("yaml_text", "message"),
    [
        ("sim: {}\n", "objects"),
        ("objects: []\n", "non-empty"),
        (
            "objects:\n  - name: banana\n  - name: BANANA\n",
            "duplicate.*banana",
        ),
        ("objects:\n  - type: mesh\n", "valid name"),
    ],
)
def test_load_available_object_names_rejects_invalid_yaml_config(
    launch_module,
    tmp_path,
    yaml_text,
    message,
):
    config_path = tmp_path / "base_env.yaml"
    config_path.write_text(yaml_text, encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        launch_module.load_available_object_names(config_path)


def test_parse_assigned_objects_trims_spaces_and_ignores_case(launch_module):
    available = ("banana", "apple", "foam_brick")

    assert launch_module.parse_assigned_object_names(
        " Banana,apple , FOAM_BRICK ",
        available,
    ) == ["banana", "apple", "foam_brick"]


def test_parse_assigned_objects_allows_blank_line(launch_module):
    assert launch_module.parse_assigned_object_names(
        "   ",
        ("banana", "apple"),
    ) == []


def test_explicit_assigned_objects_bypass_prompt(launch_module):
    def fail_if_prompted(_prompt):
        raise AssertionError("explicit assigned names must not prompt")

    assert launch_module.resolve_assigned_object_argument(
        "[ Banana,APPLE ]",
        ("banana", "apple", "foam_brick"),
        input_fn=fail_if_prompted,
    ) == ["banana", "apple"]


def test_explicit_empty_assigned_objects_bypass_prompt(launch_module):
    def fail_if_prompted(_prompt):
        raise AssertionError("explicit empty assignment must not prompt")

    assert launch_module.resolve_assigned_object_argument(
        "[]",
        ("banana", "apple"),
        input_fn=fail_if_prompted,
    ) == []


def test_default_assigned_object_argument_prompts_once(launch_module):
    prompts = []

    assert launch_module.resolve_assigned_object_argument(
        "prompt",
        ("banana", "apple"),
        input_fn=lambda prompt: prompts.append(prompt) or "banana",
        output_fn=lambda _message: None,
    ) == ["banana"]
    assert len(prompts) == 1


@pytest.mark.parametrize(
    ("raw_names", "message"),
    [
        ("banana,,apple", "empty entry"),
        ("banana,BANANA", "duplicate.*banana"),
        ("banana,missing", "unknown.*missing"),
        ("a,b,c,d,e,f,g", "at most 6.*got 7"),
    ],
)
def test_parse_assigned_objects_rejects_invalid_input(
    launch_module,
    raw_names,
    message,
):
    available = ("banana", "apple", "a", "b", "c", "d", "e", "f", "g")

    with pytest.raises(ValueError, match=message):
        launch_module.parse_assigned_object_names(raw_names, available)


def test_assigned_object_prompt_retries_entire_line(launch_module):
    responses = iter(["banana,,apple", "banana, missing", "banana, APPLE"])
    messages = []

    result = launch_module.resolve_assigned_object_names(
        ("banana", "apple", "foam_brick"),
        input_fn=lambda _prompt: next(responses),
        output_fn=messages.append,
    )

    assert result == ["banana", "apple"]
    assert sum("Available objects (3)" in message for message in messages) == 3
    assert sum("Invalid assigned objects" in message for message in messages) == 2


def test_assigned_object_prompt_closed_input_fails_cleanly(launch_module):
    def closed_input(_prompt):
        raise EOFError

    with pytest.raises(RuntimeError, match="assigned-object input stream closed"):
        launch_module.resolve_assigned_object_names(
            ("banana", "apple"),
            input_fn=closed_input,
            output_fn=lambda _message: None,
        )


def test_assign_launch_prompts_and_forwards_hydra_list(
    launch_module,
    monkeypatch,
):
    prompted_with = []
    monkeypatch.setattr(
        launch_module,
        "load_available_object_names",
        lambda: ("banana", "apple", "foam_brick"),
    )
    monkeypatch.setattr(
        launch_module,
        "resolve_assigned_object_names",
        lambda available_names, **_kwargs: prompted_with.append(
            tuple(available_names)
        )
        or ["banana", "apple"],
    )

    class FakeContext:
        responses = iter(["assign", "prompt", "false"])

        @classmethod
        def perform_substitution(cls, _substitution):
            return next(cls.responses)

    actions = launch_module.launch_mujoco_ros_interface(FakeContext())
    shell_command = "".join(
        substitution.text for substitution in actions[0].cmd[2]
    )

    assert prompted_with == [("banana", "apple", "foam_brick")]
    assert "scene_mode=assign" in shell_command
    assert "assigned_object_names=[banana,apple]" in shell_command


def test_assign_launch_uses_explicit_names_without_prompt(
    launch_module,
    monkeypatch,
):
    monkeypatch.setattr(
        launch_module,
        "load_available_object_names",
        lambda: ("banana", "apple", "foam_brick"),
    )

    def fail_if_prompted(*_args, **_kwargs):
        raise AssertionError("explicit assigned names must not prompt")

    monkeypatch.setattr(
        launch_module,
        "resolve_assigned_object_names",
        fail_if_prompted,
    )

    class FakeContext:
        responses = iter(["assign", "[banana,apple]", "false"])

        @classmethod
        def perform_substitution(cls, _substitution):
            return next(cls.responses)

    actions = launch_module.launch_mujoco_ros_interface(FakeContext())
    shell_command = "".join(
        substitution.text for substitution in actions[0].cmd[2]
    )

    assert "assigned_object_names=[banana,apple]" in shell_command
