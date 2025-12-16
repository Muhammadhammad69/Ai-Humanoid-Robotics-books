#!/usr/bin/env python3
"""
Create OpenAI Agent using the OpenAI Agents SDK

This script demonstrates how to create an agent using OpenAI models with the official custom model provider pattern.
"""

import os
import asyncio
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel



def create_openai_agent(name="OpenAI Assistant", instructions="You are a helpful assistant.", model="gpt-4o"):
    """
    Create an OpenAI agent with the specified parameters using the official custom model provider pattern.

    Args:
        name (str): Name of the agent
        instructions (str): System instructions for the agent
        model (str): OpenAI model to use

    Returns:
        Agent: Configured OpenAI agent
    """
    # Ensure API key is set
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise ValueError("OPENAI_API_KEY environment variable must be set")

    # Create AsyncOpenAI client with API key
    client = AsyncOpenAI(api_key=api_key)

    # Create model instance using the client
    model_instance = OpenAIChatCompletionsModel(model=model, openai_client=client)

    agent = Agent(
        name=name,
        instructions=instructions,
        model=model_instance
    )

    return agent


async def run_openai_agent(agent, user_input):
    """
    Run the OpenAI agent with the given input.

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
    print("Creating OpenAI Agent...")

    # Create agent
    agent = create_openai_agent(
        name="OpenAI Assistant",
        instructions="You are a helpful assistant. Be concise and friendly in your responses.",
        model="gpt-4o"
    )

    print(f"Agent '{agent.name}' created with model '{agent.model}'")
    print("Agent ready to process requests.")


if __name__ == "__main__":
    main()