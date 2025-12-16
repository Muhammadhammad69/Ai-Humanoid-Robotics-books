#!/usr/bin/env python3
"""
Create Custom LLM Agent using the OpenAI Agents SDK

This script demonstrates how to create an agent using custom LLM providers
through the official custom model provider pattern.
"""

import os
import asyncio
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel



def create_custom_agent(base_url, api_key, name="Custom Assistant", instructions="You are a helpful assistant.", model="custom-model"):
    """
    Create a custom LLM agent with the specified parameters using the official custom model provider pattern.

    Args:
        base_url (str): Base URL for the custom LLM API
        api_key (str): API key for the custom LLM service
        name (str): Name of the agent
        instructions (str): System instructions for the agent
        model (str): Model name to use

    Returns:
        Agent: Configured custom agent
    """
    # Create AsyncOpenAI client with custom base URL and API key
    client = AsyncOpenAI(base_url=base_url, api_key=api_key)

    # Create model instance using the custom client
    model_instance = OpenAIChatCompletionsModel(model=model, openai_client=client)

    agent = Agent(
        name=name,
        instructions=instructions,
        model=model_instance
    )

    return agent


def create_anthropic_agent(name="Anthropic Assistant", instructions="You are a helpful assistant.", model="claude-3-5-sonnet-20241022"):
    """
    Create an Anthropic agent with the specified parameters using the official custom model provider pattern.

    Args:
        name (str): Name of the agent
        instructions (str): System instructions for the agent
        model (str): Anthropic model to use

    Returns:
        Agent: Configured Anthropic agent
    """
    base_url = os.getenv("ANTHROPIC_BASE_URL", "https://openai-proxy.berri.ai/anthropic/v1")
    api_key = os.getenv("ANTHROPIC_API_KEY")

    if not api_key:
        raise ValueError("ANTHROPIC_API_KEY environment variable must be set")

    # Create AsyncOpenAI client with custom base URL and API key for Anthropic proxy
    client = AsyncOpenAI(base_url=base_url, api_key=api_key)

    # Create model instance using the custom client
    model_instance = OpenAIChatCompletionsModel(model=model, openai_client=client)

    agent = Agent(
        name=name,
        instructions=instructions,
        model=model_instance
    )

    return agent


def create_ollama_agent(name="Ollama Assistant", instructions="You are a helpful assistant.", model="llama3.1"):
    """
    Create an Ollama agent with the specified parameters using the official custom model provider pattern.

    Args:
        name (str): Name of the agent
        instructions (str): System instructions for the agent
        model (str): Ollama model to use

    Returns:
        Agent: Configured Ollama agent
    """
    base_url = os.getenv("OLLAMA_BASE_URL", "http://localhost:11434/v1")
    api_key = os.getenv("OLLAMA_API_KEY", "")  # Ollama typically doesn't require an API key

    # Create AsyncOpenAI client with custom base URL for Ollama
    client = AsyncOpenAI(base_url=base_url, api_key=api_key)

    # Create model instance using the custom client
    model_instance = OpenAIChatCompletionsModel(model=model, openai_client=client)

    agent = Agent(
        name=name,
        instructions=instructions,
        model=model_instance
    )

    return agent


async def run_custom_agent(agent, user_input):
    """
    Run the custom agent with the given input.

    Args:
        agent (Agent): The agent to run
        user_input (str): User input to process

    Returns:
        str: Agent's response
    """
    result = await Runner.run(agent, user_input)
    return result.final_output


def main():
    # Example usage
    print("Creating Custom Agent...")

    # Example for a generic custom LLM
    try:
        agent = create_custom_agent(
            base_url="https://your-custom-llm-provider.com/v1",
            api_key="your-api-key",
            name="Custom Assistant",
            instructions="You are a helpful assistant. Be concise and friendly in your responses.",
            model="your-model-name"
        )

        print(f"Custom agent '{agent.name}' created with model '{agent.model}'")
        print("Agent ready to process requests.")
    except ValueError as e:
        print(f"Error creating custom agent: {e}")

    # Example for Ollama (if running locally)
    try:
        ollama_agent = create_ollama_agent(
            name="Ollama Assistant",
            instructions="You are a helpful assistant. Be concise and friendly in your responses.",
            model="llama3.1"
        )
        print(f"Ollama agent '{ollama_agent.name}' created with model '{ollama_agent.model}'")
        print("Note: Make sure Ollama is running at http://localhost:11434")
    except ValueError as e:
        print(f"Error creating Ollama agent: {e}")


if __name__ == "__main__":
    main()