"""Shared HTTP retry behavior for remote perception services."""

import time
from collections.abc import Callable
from typing import Any

import requests


RETRYABLE_STATUS_CODES = {429, 500, 502, 503, 504}


def short_response_text(response: Any) -> str:
    """Return a compact response body for human-readable request errors."""
    if response is None:
        return ""

    text = getattr(response, "text", None)
    if text is None:
        try:
            text = response.read().decode("utf-8", errors="replace")
        except Exception:
            text = str(response)
    return " ".join(str(text).split())[:300]


def post_with_retries(
    url: str,
    *,
    attempts: int = 3,
    response_text: Callable[[Any], str] = short_response_text,
    retryable_status_codes: set[int] = RETRYABLE_STATUS_CODES,
    **kwargs,
) -> requests.Response:
    """POST with the existing exponential retry and error-reporting contract."""
    last_error = None
    for attempt in range(1, attempts + 1):
        try:
            response = requests.post(url, **kwargs)
            if response.status_code not in retryable_status_codes:
                response.raise_for_status()
                return response
            last_error = RuntimeError(
                f"HTTP {response.status_code}: {response_text(response)}"
            )
        except requests.RequestException as exc:
            last_error = exc

        if attempt < attempts:
            wait_sec = 2 ** (attempt - 1)
            print(f"Request to {url} failed ({last_error}); retrying in {wait_sec}s...")
            time.sleep(wait_sec)

    raise RuntimeError(
        f"Request to {url} failed after {attempts} attempts. Last error: {last_error}"
    ) from last_error
