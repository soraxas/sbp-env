# Contributing to `sbp-env`

We love your input! We welcome anyone make any form of contribution to `sbp-env`, whether it's:

- Reporting a bug
- Discussing the current state of the code
- Submitting a fix
- Proposing new features
- Becoming a maintainer

## Continuous integration with Github

We use Github to host code, track issues, pull requests, run automated test, as well as building documentations.

## Pull Requests

Pull requests are the best way to propose changes to the codebase:

1. Fork the repo and create your branch from `master`.
2. If you've added code that should be tested, add tests.
3. If you've changed APIs, update the documentation.
4. Ensure the test suite passes.
5. Ensure the documentations are adequate.
6. Make sure your code lints (we use `black` for formatting).
7. Issue that pull request!

## Report bugs using Github's [issues](https://github.com/soraxas/sbp-env/issues)

We use GitHub issues to track public bugs. Report a bug by [opening a new issue](https://github.com/soraxas/sbp-env/issues); it's that easy!

## Write bug reports with detail, background, and sample code

**Great Bug Reports** tend to have:

- A quick summary and/or background
- Steps to reproduce
  - Be specific!
- What you expected would happen
- What actually happens

## Testing the contributing code

You can check for all test cases with
```sh
pytest tests
```
in the root of the repository.

## Use a Consistent Coding Style

We use [Python Black](https://github.com/psf/black) for code formatting.

## Write documentations

We use [Sphinx](https://www.sphinx-doc.org/en/master/) to automatically build documentation directly from docstring. If you want to add more in-depth guide for any additional feature, feel free to add extra `.rst` file under `docs/` folder.

## License

By contributing, you agree that your contributions will be licensed under its MIT License.
