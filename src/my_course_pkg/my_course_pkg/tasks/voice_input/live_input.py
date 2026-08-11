"""Shared live-microphone instruction input for course demos."""

import html
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import tempfile
import threading
from urllib.parse import urlsplit
import webbrowser


def add_instruction_arguments(parser):
    parser.add_argument(
        "-i",
        "--instruction",
        help="Text instruction. If omitted, prompt interactively.",
    )
    parser.add_argument(
        "--voice",
        action="store_true",
        help="Record the instruction from the microphone.",
    )
    parser.add_argument(
        "--voice-seconds",
        type=int,
        default=int(os.environ.get("VOICE_RECORD_SECONDS", "5")),
        help="Microphone recording duration.",
    )
    parser.add_argument(
        "--voice-model",
        default=os.environ.get(
            "VOICE_TRANSCRIPTION_MODEL",
            "kit.whisper-large-v3",
        ),
        help="OpenAI-compatible transcription model.",
    )
    parser.add_argument(
        "--voice-language",
        default=os.environ.get("VOICE_LANGUAGE"),
        help="Optional language hint such as zh, en or de.",
    )
    parser.add_argument(
        "--voice-input",
        choices=("auto", "browser", "alsa"),
        default=os.environ.get("VOICE_INPUT", "auto"),
        help=(
            "Microphone source. 'auto' uses ALSA when a capture device exists "
            "and otherwise opens a browser microphone page."
        ),
    )
    parser.add_argument(
        "--voice-port",
        type=int,
        default=int(os.environ.get("VOICE_BROWSER_PORT", "8765")),
        help="Port for browser microphone capture.",
    )
    parser.add_argument(
        "--voice-wait-seconds",
        type=int,
        default=int(os.environ.get("VOICE_BROWSER_WAIT_SECONDS", "180")),
        help="Maximum time to wait for browser microphone input.",
    )
    return parser


def alsa_capture_available():
    """Return whether arecord can see at least one capture sound card."""
    if shutil.which("arecord") is None:
        return False
    result = subprocess.run(
        ["arecord", "-l"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    return result.returncode == 0 and "card " in result.stdout.lower()


def record_alsa_instruction(duration_sec):
    if int(duration_sec) <= 0:
        raise ValueError("Voice recording duration must be positive.")
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as wav_file:
        wav_path = Path(wav_file.name)
    try:
        print(f"Recording voice instruction for {int(duration_sec)}s...")
        subprocess.run(
            [
                "arecord",
                "-q",
                "-d",
                str(int(duration_sec)),
                "-f",
                "cd",
                "-t",
                "wav",
                str(wav_path),
            ],
            check=True,
        )
        return wav_path
    except FileNotFoundError as exc:
        wav_path.unlink(missing_ok=True)
        raise RuntimeError(
            "Voice input needs 'arecord' from the alsa-utils package."
        ) from exc
    except subprocess.CalledProcessError as exc:
        wav_path.unlink(missing_ok=True)
        raise RuntimeError(
            "Voice recording failed; check container microphone access."
        ) from exc


def _browser_capture_page(token, duration_sec):
    safe_token = html.escape(token, quote=True)
    duration_ms = int(duration_sec) * 1000
    return f"""<!doctype html>
<html lang="en">
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot voice instruction</title>
<style>
body {{ font: 18px system-ui, sans-serif; max-width: 36rem; margin: 4rem auto;
       padding: 0 1.2rem; color: #18202a; }}
button {{ font: inherit; padding: .8rem 1.2rem; cursor: pointer; }}
#status {{ margin-top: 1.2rem; }}
</style>
<h1>Robot voice instruction</h1>
<p>Press the button and speak naturally. The language is detected automatically.</p>
<button id="record">Start recording</button>
<div id="status">Ready</div>
<script>
const button = document.querySelector("#record");
const status = document.querySelector("#status");
button.onclick = async () => {{
  button.disabled = true;
  try {{
    const stream = await navigator.mediaDevices.getUserMedia({{audio: true}});
    const preferred = "audio/webm;codecs=opus";
    const options = MediaRecorder.isTypeSupported(preferred)
      ? {{mimeType: preferred}} : undefined;
    const recorder = new MediaRecorder(stream, options);
    const chunks = [];
    recorder.ondataavailable = event => {{
      if (event.data.size) chunks.push(event.data);
    }};
    recorder.onstop = async () => {{
      stream.getTracks().forEach(track => track.stop());
      status.textContent = "Transcribing...";
      const blob = new Blob(chunks, {{type: recorder.mimeType || "audio/webm"}});
      const response = await fetch("/capture/{safe_token}", {{
        method: "POST",
        headers: {{"Content-Type": blob.type}},
        body: blob
      }});
      if (!response.ok) throw new Error(await response.text());
      status.textContent = "Voice instruction received. You may close this page.";
    }};
    status.textContent = "Recording...";
    recorder.start();
    setTimeout(() => recorder.stop(), {duration_ms});
  }} catch (error) {{
    status.textContent = "Microphone error: " + error.message;
    button.disabled = false;
  }}
}};
</script>
</html>"""


def record_browser_instruction(
    duration_sec,
    port=8765,
    timeout_sec=180,
    on_ready=None,
    open_browser=True,
):
    """Capture live audio in the host browser and return a temporary WebM."""
    duration_sec = int(duration_sec)
    port = int(port)
    timeout_sec = int(timeout_sec)
    if duration_sec <= 0:
        raise ValueError("Voice recording duration must be positive.")
    if not 1 <= port <= 65535:
        raise ValueError("Voice browser port must be between 1 and 65535.")
    if timeout_sec <= 0:
        raise ValueError("Voice browser wait time must be positive.")

    token = secrets.token_urlsafe(24)
    received = threading.Event()
    state = {"path": None, "error": None}
    page = _browser_capture_page(token, duration_sec).encode("utf-8")

    class CaptureHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            request_path = urlsplit(self.path).path
            if request_path not in {"/", f"/{token}"}:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def do_POST(self):
            if urlsplit(self.path).path != f"/capture/{token}":
                self.send_error(404)
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 25 * 1024 * 1024:
                    raise ValueError("Invalid audio upload size.")
                payload = self.rfile.read(length)
                with tempfile.NamedTemporaryFile(
                    suffix=".webm",
                    delete=False,
                ) as audio_file:
                    audio_file.write(payload)
                    state["path"] = Path(audio_file.name)
                self.send_response(204)
                self.end_headers()
            except Exception as exc:
                state["error"] = exc
                self.send_error(400, str(exc))
            finally:
                received.set()

        def log_message(self, _format, *_args):
            return

    try:
        server = ThreadingHTTPServer(("0.0.0.0", port), CaptureHandler)
    except OSError as exc:
        raise RuntimeError(
            f"Could not start browser microphone server on port {port}."
        ) from exc
    server_thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )
    server_thread.start()
    try:
        url = f"http://localhost:{port}"
        if on_ready is not None:
            on_ready(url)
        print(
            "\nLive microphone capture is ready.\n"
            f"Open {url} in the host browser, "
            "allow microphone access, and press Start recording.\n"
        )
        if open_browser:
            try:
                if not webbrowser.open_new_tab(url):
                    print(f"Could not open a browser automatically. Use {url}.")
            except Exception as exc:
                print(f"Could not open a browser automatically ({exc}). Use {url}.")
        if not received.wait(timeout_sec):
            raise TimeoutError(
                "Timed out waiting for browser microphone input."
            )
        if state["error"] is not None:
            raise RuntimeError("Browser audio capture failed.") from state["error"]
        if state["path"] is None:
            raise RuntimeError("Browser returned no audio.")
        return state["path"]
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=2.0)


def record_voice_instruction(
    duration_sec,
    input_mode="auto",
    browser_port=8765,
    browser_timeout_sec=180,
):
    """Record from ALSA or from the host browser."""
    input_mode = str(input_mode)
    if input_mode == "alsa":
        return record_alsa_instruction(duration_sec)
    if input_mode == "browser":
        return record_browser_instruction(
            duration_sec,
            port=browser_port,
            timeout_sec=browser_timeout_sec,
        )
    if input_mode != "auto":
        raise ValueError(f"Unsupported voice input mode: {input_mode}")
    if alsa_capture_available():
        return record_alsa_instruction(duration_sec)
    print("No ALSA capture device found; using the host browser microphone.")
    return record_browser_instruction(
        duration_sec,
        port=browser_port,
        timeout_sec=browser_timeout_sec,
    )


def transcribe_audio_instruction(audio_path, model, language=None):
    from openai import OpenAI

    audio_path = Path(audio_path)
    if not audio_path.is_file():
        raise FileNotFoundError(f"Voice instruction file not found: {audio_path}")
    api_key = (
        os.environ.get("VOICE_API_KEY")
        or os.environ.get("VLM_API_KEY")
        or os.environ.get("OPENAI_API_KEY")
    )
    if not api_key:
        raise RuntimeError(
            "Voice transcription needs VOICE_API_KEY, VLM_API_KEY or "
            "OPENAI_API_KEY."
        )
    base_url = (
        os.environ.get("VOICE_BASE_URL")
        or os.environ.get("VLM_BASE_URL")
        or os.environ.get("OPENAI_BASE_URL")
        or "https://ki-toolbox.scc.kit.edu/api/v1"
    )
    kwargs = {"model": str(model)}
    if language:
        kwargs["language"] = str(language)
    with audio_path.open("rb") as audio_file:
        result = OpenAI(
            api_key=api_key,
            base_url=base_url,
        ).audio.transcriptions.create(
            file=audio_file,
            **kwargs,
        )
    text = result if isinstance(result, str) else result.text
    text = str(text).strip()
    if not text:
        raise RuntimeError("Voice transcription returned an empty instruction.")
    return text


def instruction_from_args(args, prompt="Instruction: "):
    if args.instruction and args.voice:
        raise ValueError("Use either --instruction or --voice, not both.")
    if args.voice:
        audio_path = record_voice_instruction(
            args.voice_seconds,
            input_mode=args.voice_input,
            browser_port=args.voice_port,
            browser_timeout_sec=args.voice_wait_seconds,
        )
        try:
            instruction = transcribe_audio_instruction(
                audio_path,
                model=args.voice_model,
                language=args.voice_language,
            )
        finally:
            audio_path.unlink(missing_ok=True)
        print(f"Transcribed instruction: {instruction}")
        return instruction
    if args.instruction:
        return str(args.instruction).strip()
    return input(prompt).strip()
