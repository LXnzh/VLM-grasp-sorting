import importlib

from my_course_pkg.perception import llm_sam2


def test_vlm_model_defaults_to_gpt_5_mini(monkeypatch):
    with monkeypatch.context() as context:
        context.delenv("VLM_MODEL", raising=False)
        module = importlib.reload(llm_sam2)
        assert module.VLM_MODEL == "azure.gpt-5-mini"

    importlib.reload(llm_sam2)


def test_vlm_model_environment_override_is_preserved(monkeypatch):
    with monkeypatch.context() as context:
        context.setenv("VLM_MODEL", "test.vision-model")
        module = importlib.reload(llm_sam2)
        assert module.VLM_MODEL == "test.vision-model"

    importlib.reload(llm_sam2)
