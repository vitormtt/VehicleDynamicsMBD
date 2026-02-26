# Contributing to VehicleDynamicsMBD-

Thank you for your interest in contributing to this project!
Please follow the guidelines below to keep the codebase consistent,
reviewable, and maintainable.

---

## Table of Contents

1. [Branching Strategy](#branching-strategy)
2. [Commit Messages](#commit-messages)
3. [MATLAB / Simulink Coding Guidelines](#matlab--simulink-coding-guidelines)
4. [Python Side-Script Guidelines](#python-side-script-guidelines)
5. [Docstring Requirements](#docstring-requirements)
6. [Pull Request Checklist](#pull-request-checklist)

---

## Branching Strategy

| Branch prefix | Purpose |
|---|---|
| `main` | Stable, release-ready code only |
| `develop` | Integration branch for completed features |
| `feature/<short-description>` | New features or enhancements |
| `fix/<short-description>` | Bug fixes |
| `docs/<short-description>` | Documentation-only changes |

Branch names must be lowercase and use hyphens, not underscores.

```
# Good
git checkout -b feature/mpc-controller
git checkout -b fix/roll-angle-saturation

# Bad
git checkout -b Feature_MPC
```

---

## Commit Messages

Follow the **Conventional Commits** specification
(<https://www.conventionalcommits.org/>):

```
<type>(<scope>): <short summary in imperative mood>

[optional body: explain *why*, not *what*]

[optional footer: BREAKING CHANGE or issue reference]
```

**Types:** `feat`, `fix`, `docs`, `refactor`, `test`, `chore`, `perf`

```
# Examples
feat(controllers): add LQR gain scheduling for varying speed
fix(scenarios): correct fishhook steering ramp timing
docs(readme): update quick-start instructions
```

---

## MATLAB / Simulink Coding Guidelines

This project follows the **MATLAB Advisory Board (MAB) guidelines**
for Model-Based Design. Key rules:

### General MATLAB (`*.m`)

- Use **4-space indentation** (no tabs).
- Limit line length to **80 characters** where practical.
- Use `camelCase` for variable names and `PascalCase` for class names.
- Every function file must begin with an **H1 comment line** (used by
  `help` and `lookfor`).
- Avoid `eval`, `feval` with string literals, and `global` variables unless
  strictly necessary; document any exceptions.
- Prefer vectorised operations over `for` loops where performance matters.

### Simulink models (`*.slx`)

- Organise each model into **subsystems** with clear, descriptive names.
- Use **Data Dictionaries** (`.sldd`) for all tunable parameters; do not
  hard-code numerical values in blocks.
- Signal lines must be **named** when they cross subsystem boundaries.
- Enable **strict bus** mode; avoid untyped/virtual buses.
- All subsystems must include an **inport / outport description** in the
  block description field.
- Store model configuration sets in a shared `*.m` or `.sldd` file; do
  not commit per-model solver settings that differ from the project default.
- Run **Model Advisor** checks (`Modeling Standards for MAB`) before
  opening a Pull Request and fix all warnings in the *Required* category.

---

## Python Side-Script Guidelines

Python utility scripts (post-processing, data wrangling, plotting) must
conform to **PEP 8** (<https://peps.python.org/pep-0008/>):

- 4-space indentation; no tabs.
- Maximum line length of **79 characters** (use `black` or `flake8`).
- Use **snake_case** for variables and functions, `PascalCase` for classes.
- Imports must appear at the top of the file, grouped as:
  1. Standard library
  2. Third-party packages
  3. Local modules
- All public functions must have a **docstring** (see next section).
- Run `flake8` (or `ruff`) before committing:

```bash
pip install flake8
flake8 scripts/
```

---

## Docstring Requirements

### MATLAB functions

Every `.m` function file **must** include:

```matlab
function output = my_function(input1, input2)
%MY_FUNCTION One-line summary (H1 line, all caps function name).
%
%   OUTPUT = MY_FUNCTION(INPUT1, INPUT2) explains what the function does,
%   its inputs, outputs, and any important assumptions.
%
%   Inputs:
%       input1 - Description, units, and valid range.
%       input2 - Description, units, and valid range.
%
%   Outputs:
%       output - Description and units of the returned value.
%
%   Example:
%       result = my_function(1.0, [0; 0; 0]);
%
%   See also: RELATED_FUNCTION, OTHER_FUNCTION.
```

### Python functions

Every public Python function must include a **Google-style** docstring:

```python
def compute_roll_angle(lateral_accel: float, speed: float) -> float:
    """Compute estimated steady-state roll angle.

    Args:
        lateral_accel: Lateral acceleration in m/s².
        speed: Vehicle speed in m/s.

    Returns:
        Steady-state roll angle in radians.

    Raises:
        ValueError: If speed is negative.
    """
```

---

## Pull Request Checklist

Before requesting a review, ensure:

- [ ] `setup_environment.m` runs without errors on a clean MATLAB session.
- [ ] All new/modified `.m` functions include compliant docstrings.
- [ ] Simulink Model Advisor passes with no *Required* warnings.
- [ ] `flake8` reports no errors on changed Python scripts.
- [ ] New functionality is accompanied by tests in `tests/`.
- [ ] `results/` and `build/` directories are **not** included in the commit
      (they are git-ignored).
- [ ] Large data files (> 10 MB) are **not** committed; use Git LFS or an
      external data store instead.
- [ ] The PR description explains *what* changed and *why*.
