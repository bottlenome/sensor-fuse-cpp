# Contribution Guidelines

This repository contains C++ sources that should compile with at least C++17.

## Style
- Format all C++ code using `clang-format` with the Google style.
- Keep line lengths under 100 characters.
- Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).

## Testing
- Build using CMake: `cmake -S . -B build` followed by `cmake --build build`.
- Run tests (if any) with `ctest --test-dir build`.

## Documentation
 - Document public interfaces in headers using Doxygen style comments.
 - Update `README.md` when adding new features or build steps.

## CI
 - Workflows reside in `.github/workflows`.

## Commit messages
- Use short imperative summaries.

