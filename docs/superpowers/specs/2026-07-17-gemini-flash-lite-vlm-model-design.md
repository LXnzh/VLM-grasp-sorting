# Gemini Flash Lite VLM Model Migration Design

## Goal

Restore the image-based target-selection stage after the KI-Toolbox endpoint
stopped exposing the current default model, while keeping external quota usage
low and leaving segmentation and robot motion unchanged.

## Confirmed Failure

The pipeline currently defaults to `azure.gpt-4.1-mini` at
`https://ki-toolbox.scc.kit.edu/api/v1`. On 2026-07-17 the same credential used
by the experiment supervisor successfully queried `GET /models` with HTTP 200,
but the returned 31-model catalog did not contain that ID. The subsequent chat
completion therefore failed with HTTP 400 `Model not found` before SAM2 or any
motion ran.

## Initially Selected Approach

Change only the default `VLM_MODEL` value to
`google.gemini-2.5-flash-lite`. Keep the existing `VLM_MODEL` environment
override, API base URL, credential lookup, retry behavior, prompt, JSON parsing,
SAM2 request, and all grasp behavior unchanged.

The endpoint metadata marks this model active with vision input and text output.
Its listed price is USD 0.10 per million input tokens and USD 0.40 per million
output tokens, making it a low-cost external option. It avoids selecting the
cheapest `azure.gpt-5-nano`, whose lower capability and possible GPT-5 request-
parameter compatibility changes would expand the repair beyond a model swap.
`azure.gpt-5-mini` offers more capability but costs materially more for a task
limited to selecting from 18 named YCB objects.

## Data Flow And Error Handling

The saved RGB image and instruction continue through the existing OpenAI-
compatible `chat.completions` call. A successful response must still parse as
JSON and resolve to allowed YCB object names. Existing failures remain
fail-closed: an unavailable model, unsupported image input, malformed response,
or exhausted retry budget stops the pipeline before SAM2 and motion.

## Verification

1. Add or update focused configuration coverage proving that the unoverridden
   implemented default is used and that `VLM_MODEL` still overrides it.
2. Run Python compilation and the focused VLM tests.
3. Use the already captured RGB image for one direct candidate-selection call
   with the supervisor credential. Require a successful response containing
   only allowed YCB object names.
4. Do not invoke SAM2, the grasp demo, a reset, or any robot-motion interface as
   part of this verification.

## Rollback

No code rollback is needed to try another supported model: set `VLM_MODEL` in
the launch environment. Any future default must first pass the same direct
image-request compatibility check used here.

## Validation Outcome

The direct Flash Lite image request reached KI-Toolbox but failed at its Google
Vertex EU backend with HTTP 404: the publisher model was unavailable in the
configured project/region despite appearing active in `GET /models`. The
reviewed fallback `azure.gpt-5-mini` accepted the unchanged image request,
including `temperature=0`, and returned `{"candidates": ["apple"]}` for the
captured RGB frame and instruction. The implemented default is therefore
`azure.gpt-5-mini`; all other scope and safety constraints above remain
unchanged.
