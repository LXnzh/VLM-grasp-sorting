def _normalize_category(value):
    return str(value).strip().lower().replace("-", "_").replace(" ", "_")


def classify_food(img_path, target_object, run_vlm_json):
    """Use a VLM to decide whether an arbitrary visible target is food."""
    prompt = f"""
You are the semantic sorting module for a robot.

Look at the image and classify only the target object named: {target_object!r}

Choose exactly one category:
- food: something edible or drinkable, an ingredient, or a packaged food/beverage product
- non_food: every other object

Important rules:
1. Infer the category from the image and general semantic knowledge.
2. Do not use or assume a predefined object catalog.
3. Ignore the robot, table, and sorting bins.
4. If visual evidence is incomplete, still choose the most likely category and lower confidence.
5. Output only valid JSON in this exact shape:
{{"category": "food", "confidence": 0.0, "reason": "short reason"}}
"""
    result = run_vlm_json(img_path, prompt, "food-classification")
    category = _normalize_category(result.get("category", ""))
    if not category:
        category = "unknown"

    try:
        confidence = float(result.get("confidence", 0.0))
    except (TypeError, ValueError) as exc:
        raise ValueError(f"Invalid VLM classification confidence: {result}") from exc
    confidence = min(1.0, max(0.0, confidence))
    reason = " ".join(str(result.get("reason", "")).split())[:200]
    return {
        "category": category,
        "confidence": confidence,
        "reason": reason,
    }
