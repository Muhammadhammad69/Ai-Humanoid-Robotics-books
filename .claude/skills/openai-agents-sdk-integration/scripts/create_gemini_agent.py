#!/usr/bin/env python3
"""
Create Gemini Agent using the OpenAI Agents SDK

This script demonstrates how to create an agent using Google's Gemini models
through the official custom model provider pattern.
"""

import os
import asyncio
from openai import AsyncOpenAI
from agents import Agent, Runner, OpenAIChatCompletionsModel



def create_gemini_agent(name="Gemini Assistant", instructions="You are a helpful assistant.", model="gemini-1.5-pro"):
    """
    Create a Gemini agent with the specified parameters using the official custom model provider pattern.

    Args:
        name (str): Name of the agent
        instructions (str): System instructions for the agent
        model (str): Gemini model to use

    Returns:
        Agent: Configured Gemini agent
    """
    # Ensure API key is set
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("OPENAI_API_KEY")
    if not api_key:
        raise ValueError("GEMINI_API_KEY or OPENAI_API_KEY environment variable must be set")

    # Create AsyncOpenAI client with Gemini API endpoint and API key
    client = AsyncOpenAI(
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
        api_key=api_key
    )

    # Create model instance using the custom client
    model_instance = OpenAIChatCompletionsModel(model=model, openai_client=client)

    agent = Agent(
        name=name,
        instructions=instructions,
        model=model_instance
    )

    return agent


async def run_gemini_agent(agent, user_input):
    """
    Run the Gemini agent with the given input.

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
    print("Creating Gemini Agent...")

    # Create agent
    agent = create_gemini_agent(
        name="Gemini Assistant",
        instructions="You are a helpful assistant. Be concise and friendly in your responses.",
        model="gemini-1.5-pro"
    )

    print(f"Agent '{agent.name}' created with model '{agent.model}'")
    print("Agent ready to process requests.")
    print("Note: Make sure GEMINI_API_KEY or OPENAI_API_KEY is set in your environment.")


if __name__ == "__main__":
    main()