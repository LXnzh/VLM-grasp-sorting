# Racquetball Alias Expansion Design

## Goal

Make the existing racquetball perception alias work for the ordinary user
instructions `racquetball` and `blue ball`, in addition to the already
supported phrase `blue racquetball`. All three forms must select the canonical
YCB object `racquetball` while grounding and segmenting it as a blue ball.

## Problem

The current alias resolver recognizes only the exact phrase
`blue racquetball`. An instruction such as `pick up the racquetball` therefore
uses the generic VLM and SAM2 path. Although the first VLM stage can correctly
select the canonical name `racquetball`, SAM2 then receives the generic prompt
`racquetball.` and may return several visually unrelated round-object masks.
In the observed trials, the mask verifier selected the yellow-green tennis
ball instead of the blue racquetball.

## Scope

This is an exact extension of the existing deterministic racquetball alias.
The resolver will recognize these singular, space-separated phrases,
case-insensitively:

- `racquetball`
- `blue ball`
- `blue racquetball`

Every match returns the existing canonical identity and visual configuration:

- selected object name: `racquetball`;
- GroundingDINO/SAM2 prompt: `blue ball`;
- accepted SAM2 class name: `blue ball`;
- visual verification hint: a small smooth blue ball, not a yellow-green
  tennis ball.

The change does not alter FoundationPose model selection, grasp strategy,
approach-clearance thresholds, scene generation, or any non-racquetball object
path. `VLM_CANDIDATE_OVERRIDE` retains its existing precedence for explicit
debugging.

## Matching Rules

Matching remains bounded to complete words and permits normal whitespace
between `blue` and `ball` or `racquetball`. It must not accept plural,
underscore-joined, or substring lookalikes such as `racquetballs`,
`blue_ball`, or `blueberry racquetball`.

The implementation will keep one racquetball alias object and replace the
single-phrase regular expression with the simplest bounded expression that
covers the three approved phrases. It will not add a general alias framework,
synonym service, or compatibility branch.

## Safety Behavior

All approved phrases continue through the existing strict-alias mask path. If
multiple blue-ball masks are returned, the VLM candidate verifier must choose
one valid candidate with high confidence. If it returns low confidence, an
invalid index, or an error, the pipeline stops without using the highest-score
detection fallback and without producing a new FoundationPose result.

This change improves recall of the correct blue object but does not weaken the
existing scene-clearance gate. A later geometry mismatch will still stop before
motion.

## Validation

Focused tests will establish that:

- instructions containing each approved phrase resolve to canonical
  `racquetball`;
- each approved phrase produces the SAM2 text prompt `blue ball.` and writes
  the existing audit metadata;
- rejected near-matches do not activate the alias;
- `VLM_CANDIDATE_OVERRIDE` still takes precedence;
- ambiguous or failed alias-mask verification remains fail-closed;
- the canonical racquetball mesh is used after a valid blue-ball mask is
  selected;
- non-racquetball perception behavior remains unchanged.

Validation will begin with non-actuating unit tests and saved-frame perception
checks. Any later runtime validation must confirm the selected mask is the
actual blue racquetball before plan-only or motion testing.
