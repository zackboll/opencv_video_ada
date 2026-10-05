# Testing

- Warnings remain errors.
- Use real Core handles; no fake opaque pointers.
- Keep synthetic textures deterministic and GUI/file/camera independent.
- Verify identity and known integer translation behavior with tolerances.
- Exercise invalid depth/channels/geometry/options and arbitrary Ada lower bounds.
- Raw C++ boundary checks must exercise the actual shim and Core bridge.
- Linux ASan/UBSan instruments the actual Video shim.
- Run Alire build/test/example commands serially; generated directories are not
  safe for concurrent mutation.
- Do not convert the presence of tests into a claim that they passed.
