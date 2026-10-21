# How to Run the MANTIS-Std Test Cases

A step-by-step guide to generating, checking, and using the standard test-case dataset. For what every field in the files means, see `README.md`.

**You need:** MATLAB (any recent version, no toolboxes) and the MANTIS-Sim folder with `software\v0.1.1\emulator` inside it.

---

## Step 1 — Open MATLAB in the right folder

1. Open MATLAB.
2. In the **Command Window**, type this and press **Enter**:

   ```matlab
   cd('C:\Users\Ryan\Desktop\MANTIS-Sim\datasets\standard_testcases')
   ```

   On a different computer, replace the path with wherever `MANTIS-Sim\datasets\standard_testcases` lives. You can also click the folder icon next to the address bar at the top of MATLAB and browse to it.

3. Check that the **Current Folder** panel on the left lists `make_standard_testcases.m`. If it does, you're in the right place.

---

## Step 2 — Generate the dataset

Type and press **Enter**:

```matlab
make_standard_testcases();
```

It takes about a minute. You'll see one line per case as it finishes:

```
MANTIS-Std v1.0.0  ->  C:\...\standard_testcases\MANTIS-Std-v1.0.0
------------------------------------------------------------------------
TC01  overhead_pass      ...MB  closest   79.9 m  peak target  ... px  (... s)
TC02  offset_pass        ...
TC03  loiter             ...
TC04  low_pass           ...
TC05  f550_pass          ...
TC06  degraded_pass      ...
------------------------------------------------------------------------
6 case(s) in C:\...\MANTIS-Std-v1.0.0. Run verify_standard_testcases() next.
```

A new folder, **`MANTIS-Std-v1.0.0`**, now holds the data.

---

## Step 3 — Check the dataset

Type and press **Enter**:

```matlab
verify_standard_testcases();
```

Every column should read `ok` and every row `PASS`:

```
case  name             integ  shape  time   truth  drop   sane    result
TC01  overhead_pass    ok     ok     ok     ok     ok     ok      PASS
...
6 of 6 case(s) passed.
```

If any row says `FAIL`, see **Troubleshooting** below.

---

## Step 4 — Look at the results

Open `MANTIS-Std-v1.0.0\previews\` in File Explorer. Each PNG is a one-page summary of a case: the flight path over the nodes, the range to each node, microphone loudness, and all six camera views at the moment the drone is most visible.

---

## Step 5 — Load a case

### Without code (MATLAB only)

1. In the **Current Folder** panel, double-click `MANTIS-Std-v1.0.0`, then double-click any `.mat` file, for example `TC01_overhead_pass.mat`.
2. Six variables appear in the **Workspace** panel: `info`, `params`, `nodes`, `audio`, `video`, `truth`.
3. Double-click a variable to open it in the **Variables** editor, then double-click a field inside it (for example `truth` → `pos`) to see the numbers in a table.
4. To plot, select a field in the Variables editor and choose a chart from the **Plots** tab at the top of the window.

### With code (MATLAB)

```matlab
d = load('MANTIS-Std-v1.0.0\TC01_overhead_pass.mat');

y      = d.audio.y;          % microphone signals, one column per node
fs     = d.audio.fs;         % sample rate, 8000 Hz
frames = d.video.frames;     % camera frames: 240 x 240 x 25 frames x 6 nodes
nodeXY = d.nodes.xy;         % node positions in meters

imshow(d.video.frames(:,:,13,1))   % node 1's camera at t = 12 s
```

### With code (Python)

Install the two required libraries once:

```
pip install numpy scipy
```

Then, from the `standard_testcases` folder:

```python
import mantis_std as ms

tc = ms.load_case("MANTIS-Std-v1.0.0/TC01_overhead_pass.mat")
y      = tc["audio"]["y"]          # (192001, 6)
frames = tc["video"]["frames"]     # (240, 240, 25, 6)
```

To list every case in one line each:

```
python mantis_std.py MANTIS-Std-v1.0.0
```

---

## Step 6 — Test a tracking algorithm

1. Give your algorithm **only** `audio`, `video`, and `nodes`. `truth` is the answer key.
2. Have it return estimate times `tEst` (seconds, one column) and positions `xyzEst` (meters, three columns: x, y, z).
3. Score it against the truth:

   **MATLAB**

   ```matlab
   truthAt = interp1(d.truth.t, d.truth.pos, tEst);
   err     = xyzEst - truthAt;
   rmse    = sqrt(mean(sum(err.^2, 2), 'omitnan'))   % root-mean-square error, m
   ```

   **Python**

   ```python
   print(ms.score(tc, t_est, xyz_est))
   ```

4. Repeat for all six cases. TC01 is the baseline; each other case changes one thing (offset track, circling, low altitude, hexacopter, two dead nodes), so differences point to a cause.

When you report a result, name the version and case, for example **"MANTIS-Std v1.0.0, TC03"**.

---

## Sharing with the team

Copy `README.md`, `HOW_TO_RUN.md`, and `mantis_std.py` into `MANTIS-Std-v1.0.0`, then share that folder. Teammates only need Steps 4–6.

Keep the original `standard_testcases` folder with the `.m` scripts: they are the only way to verify or regenerate the data.

---

## Optional settings

| Goal | Command |
|---|---|
| Rebuild everything, replacing old files | `make_standard_testcases('overwrite', true);` |
| Build only some cases | `make_standard_testcases('cases', {'TC01','TC04'}, 'overwrite', true);` |
| Skip the preview images | `make_standard_testcases('previews', false);` |
| Emulator is in a different folder | `make_standard_testcases('emulatorDir', 'C:\path\to\software\v0.1.1\emulator');` |
| Rebuild from scratch and compare to saved files | `verify_standard_testcases('regenerate', true);` |

---

## Troubleshooting

| What you see | What it means | Fix |
|---|---|---|
| `Undefined function 'make_standard_testcases'` | MATLAB isn't in the right folder | Redo Step 1 and check the Current Folder panel |
| `Could not find the MANTIS-Sim emulator` | The emulator isn't at `..\..\software\v0.1.1\emulator` | Use the `emulatorDir` option from the table above |
| `exists, skipped (overwrite=false)` | The case was already built | Nothing is wrong; add `'overwrite', true` to rebuild |
| `No index.mat in ...` when verifying | The dataset hasn't been generated | Run Step 2 first |
| `FAIL` with `content hash differs` | A file was changed or corrupted after generation | Rebuild that case with `'cases', {'TC0x'}, 'overwrite', true` |
| Any other red error | Possibly a bug in the scripts | Copy the full error message and send it to Ryan |
| Python: `No module named 'scipy'` | Libraries not installed | Run `pip install numpy scipy` |
