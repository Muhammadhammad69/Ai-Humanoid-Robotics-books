#!/usr/bin/env python3
"""
Test script to validate the HTTP query endpoint
"""
import asyncio
import httpx
import json
import time


async def test_http_endpoint():
    """
    Test the HTTP query endpoint to ensure it works end-to-end
    """
    print("Testing HTTP query endpoint...")

    # Define the query endpoint
    url = "http://localhost:8000/query"

    # Define the query payload
    payload = {
        "query": "What are the basic concepts of AI?",
        "session_id": "http-test-session-001"
    }

    try:
        # Start the server in the background (we'll assume it's already running from previous tests)
        print("Making request to query endpoint...")

        start_time = time.time()

        # Send the request using httpx
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(
                url,
                json=payload,
                headers={"Content-Type": "application/json"}
            )

        total_time = time.time() - start_time

        print(f"Response status: {response.status_code}")

        if response.status_code == 200:
            result = response.json()
            print("✅ HTTP endpoint responded successfully!")
            print(f"Query: {result['query']}")
            print(f"Context length: {len(result['context'])}")
            print(f"Number of retrieved chunks: {len(result['retrieved_chunks'])}")
            print(f"Agent answer length: {len(result['agent_answer'])}")
            print(f"Processing time: {result['processing_time']:.2f} seconds")
            print(f"Total request time: {total_time:.2f} seconds")

            # Show first retrieved chunk details
            if result['retrieved_chunks']:
                first_chunk = result['retrieved_chunks'][0]
                print(f"First chunk source: {first_chunk['source_document']}")
                print(f"First chunk relevance: {first_chunk['relevance_score']}")

            # Show agent answer preview
            if result['agent_answer']:
                print(f"Agent answer preview: {result['agent_answer'][:150]}...")

            return True
        else:
            print(f"❌ HTTP endpoint returned status {response.status_code}")
            print(f"Response: {response.text}")
            return False

    except httpx.ConnectError:
        print("❌ Could not connect to the server. Make sure it's running on http://localhost:8000")
        return False
    except Exception as e:
        print(f"❌ Error during HTTP endpoint test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


async def test_server_health():
    """
    Test the health endpoint
    """
    print("\nTesting health endpoint...")

    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.get("http://localhost:8000/health")

        if response.status_code == 200:
            print("✅ Health endpoint responded successfully!")
            return True
        else:
            print(f"❌ Health endpoint returned status {response.status_code}")
            return False

    except Exception as e:
        print(f"❌ Error during health check: {str(e)}")
        return False


async def main():
    """
    Main test function for HTTP endpoints
    """
    print("Starting HTTP endpoint validation tests...\n")

    # First test the health endpoint
    health_ok = await test_server_health()

    # Then test the query endpoint
    query_ok = await test_http_endpoint()

    if health_ok and query_ok:
        print("\n✅ All HTTP endpoint tests passed!")
        print("The FastAPI server is working correctly with the RAG pipeline.")
    else:
        print("\n❌ Some HTTP endpoint tests failed.")
        print("The server may not be running or there may be configuration issues.")


if __name__ == "__main__":
    # Note: This test requires the server to be running on localhost:8000
    # You would run: uv run main.py in another terminal before running this test
    print("To run this test, please start the server in another terminal with:")
    print("cd backend && uv run main.py")
    print("\nThen run this test script.")
    asyncio.run(main())