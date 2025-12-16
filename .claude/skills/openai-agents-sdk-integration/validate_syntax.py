#!/usr/bin/env python3
"""
Validation script for OpenAI Agents SDK Integration Skill

This script validates that all components of the skill have correct syntax.
"""

import os
import sys
import ast
from pathlib import Path

def validate_syntax(file_path):
    """Validate the syntax of a Python file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        ast.parse(content)
        return True, None
    except SyntaxError as e:
        return False, str(e)
    except Exception as e:
        return False, str(e)


def validate_all_scripts():
    """Validate syntax for all Python scripts in the skill."""
    print("Validating OpenAI Agents SDK Integration Skill...")
    print("=" * 60)

    # Find all Python files in the skill directory
    skill_path = Path(__file__).parent
    python_files = list(skill_path.rglob("*.py"))

    results = []
    for file_path in python_files:
        is_valid, error = validate_syntax(file_path)
        if is_valid:
            print(f"✓ {file_path.name} - Valid syntax")
            results.append(True)
        else:
            print(f"✗ {file_path.name} - Syntax error: {error}")
            results.append(False)

    print("\n" + "=" * 60)
    print("Validation Summary:")
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("✓ All files have valid Python syntax!")
        print("\nNote: This validation checks only syntax. To fully test the skill,")
        print("you need to have the OpenAI Agents SDK installed:")
        print("pip install openai-agents")
        return True
    else:
        print("✗ Some files have syntax errors. Please fix them.")
        return False


if __name__ == "__main__":
    success = validate_all_scripts()
    sys.exit(0 if success else 1)