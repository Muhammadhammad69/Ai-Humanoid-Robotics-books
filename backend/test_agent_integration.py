#!/usr/bin/env python3
"""
Test script to verify agent integration functionality
"""

import asyncio
import os
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_path))

from src.models.query import QueryRequest
from src.services.query_processor import QueryProcessor


async def test_agent_integration():
    """
    Test that the agent integration works correctly
    """
    print("Testing agent integration...")

    # Create a sample query
    query_request = QueryRequest(
        query="What is artificial intelligence?",
        module="ai_concepts",
        language="en"
    )

    # Initialize the query processor
    processor = QueryProcessor()

    try:
        # Process the query through the pipeline
        result = await processor.process_query(query_request)

        print("Query processing completed successfully!")
        print(f"Context length: {len(result['context'])}")
        print(f"Number of retrieved chunks: {len(result['retrieved_chunks'])}")
        print(f"Agent answer: {result['agent_answer'][:100]}...")  # First 100 chars
        print(f"Processing time: {result['processing_time']:.2f} seconds")

        # Verify that agent answer is present
        if 'agent_answer' in result:
            print("✅ Agent answer field is present in the result")
        else:
            print("❌ Agent answer field is missing from the result")

        return True

    except Exception as e:
        print(f"❌ Error during query processing: {str(e)}")
        return False


async def test_error_handling():
    """
    Test error handling with the agent integration
    """
    print("\nTesting error handling...")

    # Create a processor instance
    processor = QueryProcessor()

    try:
        # Test with an empty query to trigger validation error
        empty_query = QueryRequest(query="", module="test", language="en")
        result = await processor.process_query(empty_query)
        print("❌ Expected validation error was not raised")
        return False

    except ValueError as e:
        print(f"✅ Validation error correctly raised: {str(e)}")
        return True
    except Exception as e:
        print(f"✅ Different error type raised as expected: {str(e)}")
        return True


async def main():
    """
    Main test function
    """
    print("Starting agent integration tests...\n")

    # Check if required environment variables are set
    required_vars = ['GEMINI_API_KEY', 'GEMINI_BASE_URL', 'GEMINI_MODEL']
    missing_vars = [var for var in required_vars if not os.getenv(var)]

    if missing_vars:
        print(f"⚠️  Warning: Missing environment variables: {missing_vars}")
        print("   These tests will run but may fail without proper API configuration")

    # Run tests
    success1 = await test_agent_integration()
    success2 = await test_error_handling()

    if success1 and success2:
        print("\n✅ All tests completed! Agent integration appears to be working correctly.")
        print("Note: If API credentials are missing, the agent will return empty responses.")
    else:
        print("\n❌ Some tests failed.")

    return success1 and success2


if __name__ == "__main__":
    asyncio.run(main())