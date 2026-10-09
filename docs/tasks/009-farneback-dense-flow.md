# Task 009 — dense Farnebäck flow: safety stop

Requested: an unseeded, box-refinement `Calculate_Farneback_Flow` returning
a Core-owned full-resolution Float32 C2 Mat, flags exactly zero, with no
PyrLK-pyramid reuse or application policy. Core pin, dependencies, version,
ownership and existing APIs must remain unchanged.

**Not implemented:** source review and direct-native experiments established
an upstream CPU undefined-behavior path in required versions 4.1.0 and 4.10.0.
The task explicitly requires a safety stop if a defensible restricted contract
cannot be established. See [source evidence](../farneback-source-contract.md)
and [actual qualification status](../task009-qualification.md).

The proposed parameter subset (scale .25–.90, levels 1–8, odd window 5–63,
iterations 1–30, neighborhood 5 or 7, finite sigma .1–10) does not exclude
the reproduced failure: it occurs with default options on ordinary identity and translated images.
Increasing the minimum image size does not exclude it either.

No production C export, Ada API, flags argument, backend workaround, upstream
patch, or substitute flow field was added. Resumption requires an explicitly
approved way to guarantee corrected native execution on every supported version,
or an approved change to supported versions. Merely validating results after
the native call cannot prevent undefined behavior during the call.