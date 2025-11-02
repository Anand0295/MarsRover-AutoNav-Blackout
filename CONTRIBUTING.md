# Contributing to MarsRover-AutoNav-Blackout

Thank you for your interest in contributing to the MarsRover AutoNav Blackout project! We welcome contributions from the community to help improve autonomous Mars rover navigation during communication blackouts.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
- [Development Setup](#development-setup)
- [Submission Guidelines](#submission-guidelines)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Documentation](#documentation)
- [Community](#community)

## Code of Conduct

This project and everyone participating in it is governed by our [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you are expected to uphold this code. Please report unacceptable behavior to the project maintainers.

## Getting Started

1. **Fork the repository** to your own GitHub account
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR-USERNAME/MarsRover-AutoNav-Blackout.git
   cd MarsRover-AutoNav-Blackout
   ```
3. **Create a branch** for your changes:
   ```bash
   git checkout -b feature/your-feature-name
   ```

## How to Contribute

### Reporting Bugs

If you find a bug, please create an issue with:
- A clear, descriptive title
- Steps to reproduce the behavior
- Expected vs actual behavior
- Screenshots if applicable
- Your environment (OS, Python version, etc.)

### Suggesting Enhancements

We welcome feature requests! Please create an issue with:
- A clear description of the enhancement
- Use cases and benefits
- Any implementation ideas you have

### Pull Requests

We actively welcome your pull requests:

1. Fork the repo and create your branch from `main`
2. If you've added code, add tests that cover your changes
3. Update documentation as needed
4. Ensure the test suite passes
5. Make sure your code follows our coding standards
6. Submit your pull request!

## Development Setup

### Prerequisites

- Python 3.8 or higher
- pip or conda for package management
- Git

### Installation

```bash
# Install dependencies
pip install -r requirements.txt

# Install development dependencies (if available)
pip install -r requirements-dev.txt
```

## Submission Guidelines

### Commit Messages

Write clear, concise commit messages following these guidelines:

- Use the present tense ("Add feature" not "Added feature")
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
- Limit the first line to 72 characters or less
- Reference issues and pull requests after the first line

Example:
```
Add terrain mapping feature for blackout navigation

Implements new terrain mapping algorithm that improves
navigation accuracy during communication blackouts.

Fixes #123
```

### Pull Request Guidelines

- **Keep PRs focused**: Each PR should address a single concern
- **Update documentation**: Include relevant documentation updates
- **Add tests**: Ensure new features have adequate test coverage
- **Follow style guide**: Adhere to the project's coding standards
- **Link issues**: Reference related issues in your PR description

## Coding Standards

### Python Style Guide

- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/) style guidelines
- Use meaningful variable and function names
- Add docstrings to all functions, classes, and modules
- Maximum line length: 88 characters (Black formatter standard)

### Code Formatting

We recommend using:
- **Black** for code formatting
- **isort** for import sorting
- **flake8** for linting

```bash
# Format code
black .

# Sort imports
isort .

# Run linter
flake8 .
```

## Testing

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src tests/

# Run specific test file
pytest tests/test_navigation.py
```

### Writing Tests

- Write tests for all new features and bug fixes
- Aim for high test coverage (>80%)
- Use descriptive test names that explain what is being tested
- Follow the Arrange-Act-Assert pattern

## Documentation

### Code Documentation

- Add docstrings to all public functions, classes, and modules
- Use clear, concise language
- Include parameter descriptions and return values
- Provide usage examples where helpful

Example:
```python
def calculate_trajectory(start_pos, end_pos, obstacles):
    """
    Calculate optimal trajectory from start to end position.
    
    Args:
        start_pos (tuple): Starting position (x, y, z)
        end_pos (tuple): Ending position (x, y, z)
        obstacles (list): List of obstacle positions
        
    Returns:
        list: Optimal path as list of waypoints
        
    Raises:
        ValueError: If start or end position is invalid
    """
    pass
```

### README and Guides

- Keep the README up to date with new features
- Update installation and usage instructions as needed
- Add examples for new functionality

## Community

### Getting Help

- Check existing [issues](https://github.com/Anand0295/MarsRover-AutoNav-Blackout/issues)
- Create a new issue with the "question" label
- Reach out to maintainers

### Recognition

All contributors will be recognized in our project documentation. Your contributions, whether code, documentation, bug reports, or suggestions, are valuable to this project!

## Questions?

If you have any questions about contributing, please create an issue with the "question" label or reach out to the maintainers.

---

Thank you for contributing to MarsRover-AutoNav-Blackout! 🚀
