# Contributing

Bug reports, documentation improvements, and focused pull requests are welcome. Read the [usage guide](docs/usage.md) and [implementation notes](docs/implementation-notes.md) before changing the mapping behavior.

## Report a problem

Use the [issue tracker](https://github.com/HKPolyU-UAV/EDOG-Map/issues). Include:

- Repository commit, OS, Python and ROS versions, and installed dependency versions.
- The command you ran, private ROS parameters, input topic types, and coordinate-frame conventions.
- Expected behavior, actual behavior, and the complete traceback where applicable.
- A minimal example or a shareable recording, including timestamps and calibration when needed. Keep large recordings outside Git and link to them.

State whether the issue concerns the released implementation or a result in the paper. Published benchmark results require the paper's data, preprocessing, settings, and hardware; the import and smoke checks do not reproduce those experiments.

## Submit a change

Keep each pull request focused and explain its effect on users. For algorithm changes, describe the affected evidence masses, coordinate conventions, or sensor assumptions and provide a reproducible comparison. Update the usage guide when defaults, dependencies, inputs, or outputs change.

Run the lightweight checks from the repository root in the environment described in the usage guide:

```bash
python -m compileall -q run.py libs pipeline scripts
python scripts/smoke_test.py
```

For ROS-related changes, also describe the ROS version, topics or recording, command, and observed output used for validation. The automated checks do not exercise ROS subscribers or flight integration.

Do not commit generated maps, bytecode, virtual environments, private data, or large video/rosbag files. This repository does not currently include a license file; do not add a license or redistribute third-party assets without the maintainers' decision.
