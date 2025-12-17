#!/usr/bin/env python3
"""
Direct test of the vector storage search functionality to debug the empty results issue.
"""

import sys
import os
from pathlib import Path

# Add the src directory to the Python path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.services.vector_storage import VectorStorageService
from src.services.embedding_service import EmbeddingService
from src.config.settings import Settings

async def test_search():
    print("🔍 Testing direct search functionality...")

    try:
        # Initialize services
        embedding_service = EmbeddingService()
        vector_service = VectorStorageService()
        settings = Settings()

        print(f"✅ Successfully connected to services")
        print(f"📊 Collection: {settings.QDRANT_COLLECTION_NAME}")

        # Create a test query embedding
        query_text = "What is ROS 2?"
        print(f"📝 Query: {query_text}")

        query_embedding = await embedding_service.embed_query(query_text)
        print(f"🧮 Embedding vector length: {len(query_embedding)}")

        # Perform search without filters
        print("🔍 Performing search without filters...")
        results = vector_service.search_vectors(
            query_embedding=query_embedding,
            top_k=5,
            filters={}
        )

        print(f"🎯 Found {len(results)} results")

        for i, result in enumerate(results):
            print(f"\n--- Result {i+1} ---")
            print(f"Content: '{result.content}'")  # Show content with quotes to see if empty
            print(f"Relevance Score: {result.relevance_score}")
            print(f"Source: '{result.source_document}'")
            print(f"Metadata: {result.metadata}")
            print(f"Metadata keys: {list(result.metadata.keys())}")
            print(f"Chunk ID: {result.chunk_id}")

        # Let's also try a direct search to see raw results using the same approach as the service
        print("\n🔍 Trying direct search to see raw results...")
        # Check which method is available
        if hasattr(vector_service.client, 'search'):
            print("Using 'search' method")
            raw_results = vector_service.client.search(
                collection_name=vector_service.collection_name,
                query_vector=query_embedding,
                limit=1
            )
        elif hasattr(vector_service.client, 'query_points'):
            print("Using 'query_points' method")
            raw_results = vector_service.client.query_points(
                collection_name=vector_service.collection_name,
                query=query_embedding,
                limit=1
            )
        else:
            print("❌ Neither 'search' nor 'query_points' method available")
            return results

        for i, raw_result in enumerate(raw_results):
            print(f"\n--- Raw Result {i+1} ---")
            print(f"Raw result type: {type(raw_result)}")
            print(f"Raw result: {raw_result}")
            print(f"Has payload: {hasattr(raw_result, 'payload')}")
            if hasattr(raw_result, 'payload'):
                print(f"Payload: {raw_result.payload}")
                print(f"Payload content: {raw_result.payload.get('content', 'NO CONTENT KEY')}")
            else:
                # Try to access payload in different ways
                payload = getattr(raw_result, 'payload', {})
                if not payload and hasattr(raw_result, '__dict__'):
                    payload = raw_result.__dict__.get('payload', {})
                print(f"Alternative payload access: {payload}")
            print(f"Score: {getattr(raw_result, 'score', getattr(raw_result, 'score', 'NO SCORE'))}")

        return results

    except Exception as e:
        print(f"❌ Error during search: {str(e)}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    import asyncio
    results = asyncio.run(test_search())

    if results:
        print(f"\n✅ Search completed successfully with {len(results)} results")
    else:
        print("\n❌ Search failed")