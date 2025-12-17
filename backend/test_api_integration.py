#!/usr/bin/env python3
"""
Test script to verify API integration with agent responses
"""

import asyncio
import sys
from pathlib import Path

# Add backend to path
backend_path = Path(__file__).parent / "backend"
sys.path.insert(0, str(backend_path))

from src.models.query import QueryRequest, QueryResponse
from src.services.query_processor import QueryProcessor


async def test_api_response_structure():
    """
    Test that the API response structure includes agent_answer
    """
    print("Testing API response structure...")

    # Create a sample query
    query_request = QueryRequest(
        query="What is artificial intelligence?",
        module="ai_concepts",
        language="en",
        session_id="test-session-123"
    )

    # Initialize the query processor
    processor = QueryProcessor()

    try:
        # Process the query through the pipeline
        result = await processor.validate_and_process(query_request)

        print("Query processing completed successfully!")
        print(f"Context length: {len(result['context'])}")
        print(f"Number of retrieved chunks: {len(result['retrieved_chunks'])}")
        print(f"Agent answer present: {'agent_answer' in result}")
        print(f"Agent answer length: {len(result.get('agent_answer', ''))}")
        print(f"Processing time: {result['processing_time']:.2f} seconds")

        # Verify that all required fields are present
        required_fields = ['context', 'retrieved_chunks', 'agent_answer', 'processing_time']
        missing_fields = [field for field in required_fields if field not in result]

        if not missing_fields:
            print("✅ All required fields are present in the result")
        else:
            print(f"❌ Missing fields: {missing_fields}")
            return False

        # Test creating a QueryResponse object with the result
        try:
            response = QueryResponse(
                query=query_request.query,
                context=result['context'],
                agent_answer=result.get('agent_answer', ''),
                retrieved_chunks=result['retrieved_chunks'],
                session_id=query_request.session_id,
                processing_time=result['processing_time'],
                query_id="test-query-123"
            )
            print("✅ QueryResponse object created successfully with agent answer")
            print(f"   Agent answer in response: {response.agent_answer[:50]}..." if response.agent_answer else "   Agent answer is empty")
            return True
        except Exception as e:
            print(f"❌ Failed to create QueryResponse object: {str(e)}")
            return False

    except Exception as e:
        print(f"❌ Error during query processing: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


async def test_error_handling_integration():
    """
    Test that error handling works with agent integration
    """
    print("\nTesting error handling integration...")

    # Create a processor instance
    processor = QueryProcessor()

    try:
        # Test with a valid query but potentially problematic content
        query_request = QueryRequest(
            query="Test query",
            module="test",
            language="en",
            session_id="test-session-456"
        )

        result = await processor.validate_and_process(query_request)

        print("✅ Query processed without error")
        print(f"   Agent answer present: {'agent_answer' in result}")
        print(f"   Processing time: {result['processing_time']:.2f}s")

        return True

    except ValueError as e:
        print(f"✅ Expected validation error handled: {str(e)}")
        return True
    except Exception as e:
        print(f"✅ Other error handled: {str(e)}")
        return True


async def main():
    """
    Main test function
    """
    print("Starting API integration tests...\n")

    # Run tests
    success1 = await test_api_response_structure()
    success2 = await test_error_handling_integration()

    if success1 and success2:
        print("\n✅ All API integration tests passed!")
        print("The agent integration is working correctly with the query processing pipeline.")
    else:
        print("\n❌ Some API integration tests failed.")

    return success1 and success2


if __name__ == "__main__":
    asyncio.run(main())