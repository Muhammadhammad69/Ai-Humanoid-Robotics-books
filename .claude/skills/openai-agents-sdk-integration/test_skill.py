#!/usr/bin/env python3
"""
Test script for OpenAI Agents SDK Integration Skill

This script validates that all components of the skill work correctly.
"""

import os
import sys
import asyncio
from pathlib import Path

# Add the skill scripts to the path
skill_path = Path(__file__).parent / "scripts"
sys.path.insert(0, str(skill_path))

def test_imports():
    """Test that required modules can be imported."""
    print("Testing imports...")
    try:
        # Try to import the agent creation modules
        from create_openai_agent import create_openai_agent
        from create_gemini_agent import create_gemini_agent
        from create_custom_agent import create_custom_agent, create_ollama_agent
        from multi_agent_coordinator import create_specialized_agents, create_tool_agents

        print("✓ All modules imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Import error: {e}")
        return False
    except Exception as e:
        print(f"✗ Unexpected error during import: {e}")
        return False


def test_openai_agent_creation():
    """Test OpenAI agent creation."""
    print("\nTesting OpenAI agent creation...")
    try:
        from create_openai_agent import create_openai_agent

        # Mock the API key for testing
        original_key = os.environ.get("OPENAI_API_KEY")
        os.environ["OPENAI_API_KEY"] = "test-key"

        agent = create_openai_agent(
            name="Test OpenAI Agent",
            instructions="This is a test agent.",
            model="gpt-4o"
        )

        # Verify agent properties
        assert agent.name == "Test OpenAI Agent"
        assert agent.model == "gpt-4o"

        # Restore original key
        if original_key is not None:
            os.environ["OPENAI_API_KEY"] = original_key
        else:
            del os.environ["OPENAI_API_KEY"]

        print("✓ OpenAI agent created successfully")
        return True
    except Exception as e:
        print(f"✗ OpenAI agent creation failed: {e}")
        return False


def test_gemini_agent_creation():
    """Test Gemini agent creation."""
    print("\nTesting Gemini agent creation...")
    try:
        from create_gemini_agent import create_gemini_agent

        # Mock the API key for testing
        original_key = os.environ.get("GEMINI_API_KEY")
        os.environ["GEMINI_API_KEY"] = "test-key"

        agent = create_gemini_agent(
            name="Test Gemini Agent",
            instructions="This is a test agent.",
            model="gemini-2.5-flash"
        )

        # Verify agent properties
        assert agent.name == "Test Gemini Agent"
        assert agent.model == "gemini-2.5-flash"

        # Restore original key
        if original_key is not None:
            os.environ["GEMINI_API_KEY"] = original_key
        else:
            if "GEMINI_API_KEY" in os.environ:
                del os.environ["GEMINI_API_KEY"]

        print("✓ Gemini agent created successfully")
        return True
    except Exception as e:
        print(f"✗ Gemini agent creation failed: {e}")
        return False


def test_custom_agent_creation():
    """Test custom agent creation."""
    print("\nTesting custom agent creation...")
    try:
        from create_custom_agent import create_custom_agent, create_ollama_agent

        # Test custom agent creation
        agent = create_custom_agent(
            base_url="https://test-provider.com/v1",
            api_key="test-key",
            name="Test Custom Agent",
            instructions="This is a test agent.",
            model="test-model"
        )

        # Verify agent properties
        assert agent.name == "Test Custom Agent"
        assert agent.model == "test-model"

        # Test Ollama agent creation
        ollama_agent = create_ollama_agent(
            name="Test Ollama Agent",
            instructions="This is a test agent.",
            model="llama3.1"
        )

        # Verify Ollama agent properties
        assert ollama_agent.name == "Test Ollama Agent"
        assert ollama_agent.model == "llama3.1"

        print("✓ Custom and Ollama agents created successfully")
        return True
    except Exception as e:
        print(f"✗ Custom agent creation failed: {e}")
        return False


def test_multi_agent_creation():
    """Test multi-agent coordination."""
    print("\nTesting multi-agent coordination...")
    try:
        from multi_agent_coordinator import create_specialized_agents, create_tool_agents

        # Test specialized agents creation
        agents = create_specialized_agents()
        expected_agents = {"spanish", "english", "french", "triage"}
        actual_agents = set(agents.keys())

        assert expected_agents == actual_agents, f"Expected {expected_agents}, got {actual_agents}"

        # Test tool agents creation
        tool_agents = create_tool_agents()
        expected_tool_agents = {"weather", "calculator"}
        actual_tool_agents = set(tool_agents.keys())

        assert expected_tool_agents == actual_tool_agents, f"Expected {expected_tool_agents}, got {actual_tool_agents}"

        print("✓ Multi-agent coordination components created successfully")
        return True
    except Exception as e:
        print(f"✗ Multi-agent coordination test failed: {e}")
        return False


def run_tests():
    """Run all tests."""
    print("Starting OpenAI Agents SDK Integration Skill validation...")
    print("=" * 60)

    tests = [
        test_imports,
        test_openai_agent_creation,
        test_gemini_agent_creation,
        test_custom_agent_creation,
        test_multi_agent_creation
    ]

    results = []
    for test in tests:
        results.append(test())

    print("\n" + "=" * 60)
    print("Test Summary:")
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")

    if passed == total:
        print("✓ All tests passed! The skill is ready for use.")
        return True
    else:
        print("✗ Some tests failed. Please review the implementation.")
        return False


if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)