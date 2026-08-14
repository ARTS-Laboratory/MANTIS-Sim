# Validation notes

The toolbox includes `tests/run_self_test.m`, which exercises the main signal pipeline, repeatability, range/delay consistency, reflection, a 20-microphone layout, full-node dropout, and MAT-file saving.

During package construction, the source files were checked for:

- matching MATLAB control/function blocks and `end` statements,
- balanced parentheses, brackets, braces, and quoted strings,
- presence of every required module,
- consistent documented function names and file names,
- absence of non-ASCII source-code characters.

MATLAB and GNU Octave were not installed in the build environment, so the MATLAB self-test could not be executed here. Run the following after adding the toolbox to your MATLAB path:

```matlab
results = run_self_test;
```

A successful run prints `All self-tests passed.` and returns a structure with `results.passed = true`.
