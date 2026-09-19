# Exercise template — how lesson authors add an auto-graded exercise

Copy this directory to `labs/exercises/<lesson-id>/` (the dotted id, e.g. `10.06`), then edit
the four files. `labs/exercises/09.04/` is the fully worked reference.

```text
labs/exercises/<lesson-id>/
├── README.md          what to implement, in the student's words; link back to the lesson
├── student.py         starter code: signatures, docstrings, `raise NotImplementedError("name")  # TODO(student)`
├── solution.py        reference solution with the SAME public names as student.py
└── test_exercise.py   pytest tests that use the `impl` fixture
```

## Rules

1. **Tests never import `student` or `solution` directly.** Take the `impl` fixture: it loads
   `student.py` from the test's own directory, or `solution.py` when `COURSE_USE_SOLUTION=1`
   (`python course.py check <id> --solution`).
2. **Every stub raises `NotImplementedError("<function name>")`.** A test that hits one is
   reported as *skipped — not implemented yet — edit student.py*, so the repo-wide `pytest`
   stays green. `python course.py check <id>` fails while anything is still skipped.
3. **Split the task into small functions** and test each with hand-computed cases (write the
   arithmetic in a comment), then add one or two end-to-end tests against the simulator
   (`robotlab.sim`), comparing with ground truth (`base.sim.pose`): a tight tolerance on
   `DiffDriveParams.ideal()` + `SensorParams.ideal()`, a loose one on `.realistic()` with fixed seeds.
4. **Fast and offline:** the whole exercise runs in a few seconds, no hardware, ROS or network.
5. **Read robot numbers from `load_config()`**, never hard-code karmel.yaml values, so the tests
   keep passing when a student edits the config for their robot. Hand-computed unit cases may use
   their own round numbers (e.g. `wheel_radius_m=0.05`).
6. **Failure messages teach:** add an assertion message that says what should have happened.
7. Student files use only the standard library + numpy unless the lesson is about a library.
8. Register the checker for the lesson in the curriculum (`checker: labs/exercises/<id>`) so
   `python course.py check <id>` finds it.

## Verify before committing

```bash
COURSE_USE_SOLUTION=1 python -m pytest labs/exercises/<id>   # all pass
python -m pytest labs/exercises/<id>                          # all skipped, exit code 1
python -m pytest                                              # repo-wide: green
```
