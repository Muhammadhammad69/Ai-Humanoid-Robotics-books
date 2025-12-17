#!/usr/bin/env python3
"""
Test to verify that the Qdrant search works correctly by directly accessing the raw results.
"""

import sys
import os
from pathlib import Path

# Add the src directory to the Python path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService
from src.config.settings import Settings
import asyncio

async def test_raw_qdrant_access():
    print("🔍 Testing Raw Qdrant Access")
    print("=" * 50)

    # Initialize services
    embedding_service = EmbeddingService()
    vector_service = VectorStorageService()
    settings = Settings()

    print(f"✅ Services initialized")
    print(f"📊 Collection: {settings.QDRANT_COLLECTION_NAME}")

    # Test query that should return relevant results
    test_query = "Explain rclpy communication in ROS 2"
    print(f"\n📝 Query: {test_query}")

    # Step 1: Generate embedding
    query_embedding = await embedding_service.embed_query(test_query)
    print(f"🧮 Embedding generated (length: {len(query_embedding)})")

    # Step 2: Direct search using the correct method
    print(f"\n🔍 Direct search using query_points...")

    # Check which method is available and use it properly
    if hasattr(vector_service.client, 'query_points'):
        print("Using query_points method...")
        # query_points returns (str, list_of_points) tuple
        response = vector_service.client.query_points(
            collection_name=vector_service.collection_name,
            query=query_embedding,
            limit=5
        )

        print(f"Response type: {type(response)}")
        print(f"Response: {response}")

        # The response is a tuple: (str, [ScoredPoint objects])
        if isinstance(response, tuple) and len(response) == 2:
            points_list = response[1]  # Get the list of points
            print(f"Points list type: {type(points_list)}")
            print(f"Number of points: {len(points_list)}")

            for i, point in enumerate(points_list):
                print(f"\n--- Point {i+1} ---")
                print(f"Type: {type(point)}")
                print(f"ID: {point.id}")
                print(f"Score: {point.score}")
                print(f"Payload: {point.payload}")
                print(f"Content: {point.payload.get('content', 'NO CONTENT')[:100]}...")

                # Check if content is actually there
                content = point.payload.get('content', '')
                if content:
                    print(f"✅ Content available: YES ({len(content)} chars)")
                else:
                    print(f"❌ Content available: NO")
        else:
            print(f"Unexpected response format: {response}")
    else:
        print("❌ query_points method not available")

if __name__ == "__main__":
    asyncio.run(test_raw_qdrant_access())