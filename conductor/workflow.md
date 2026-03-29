# Workflow

## Test-Driven Development (TDD)

**Policy**: Flexible - tests recommended for complex logic

- Tests are encouraged but not required for all changes
- Focus testing efforts on:
  - Complex control algorithms (PID, Kalman filter, etc.)
  - Physics simulation correctness
  - Numerical integration accuracy
  - Critical mathematical computations
- Simple UI changes and refactoring may skip tests at developer discretion

## Commit Strategy

**Conventional Commits (feat:, fix:, etc.)**

Follow the [Conventional Commits](https://www.conventionalcommits.org/) specification:

### Commit Types

- `feat:` - New feature or functionality
- `fix:` - Bug fix
- `refactor:` - Code restructuring without behavior change
- `perf:` - Performance improvement
- `docs:` - Documentation updates
- `test:` - Adding or updating tests
- `build:` - Build system or dependency changes
- `ci:` - CI/CD configuration changes
- `chore:` - Maintenance tasks

### Format

```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

### Examples

```
feat(control): add LQR controller implementation

fix(physics): correct ball rolling friction calculation

refactor(rendering): extract shader compilation into utility class

perf(kalman): optimize matrix operations with Eigen block expressions
```

## Code Review

**Policy**: Optional / self-review OK

- Code review is encouraged but not mandatory
- For solo development, self-review is acceptable
- When multiple contributors are involved, peer review is recommended
- Focus reviews on:
  - Correctness of control algorithms
  - Numerical stability
  - Performance implications
  - API design decisions

## Verification Checkpoints

**Policy**: After each task completion

- After completing each task, verify that:
  - Code compiles without warnings
  - Application runs without crashes
  - Intended functionality works as expected
  - No regressions in existing features
- Run relevant tests if available
- For UI changes, visually verify behavior
- For physics/control changes, inspect plots and numerical output

## Task Lifecycle

1. **Plan** - Define task scope and acceptance criteria
2. **Implement** - Write code following style guide
3. **Verify** - Test functionality and check for regressions
4. **Commit** - Commit with conventional commit message
5. **Document** - Update docs if public API changed

## Branch Strategy

- Single-branch development acceptable for solo work
- Feature branches recommended when collaborating
- Branch naming: `feature/<description>`, `fix/<description>`, `refactor/<description>`
