#!/usr/bin/env python3
"""
Script to check Qdrant collection stats and verify that embeddings were successfully stored.
"""

import sys
import os
from pathlib import Path

# Add the src directory to the Python path so we can import our modules
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.services.vector_storage import VectorStorageService
from src.config.settings import Settings


def main():
    """
    Main function to check Qdrant collection stats.
    """
    print("🔍 Checking Qdrant collection stats...")

    try:
        # Initialize the settings to verify environment variables are set
        settings = Settings()
        print(f"📊 Collection Name: {settings.QDRANT_COLLECTION_NAME}")
        print(f"🔗 Qdrant URL: {settings.QDRANT_URL}")
        print(f"📏 Vector Size: {settings.QDRANT_VECTOR_SIZE}")

        # Initialize the vector storage service
        vector_service = VectorStorageService()
        print("✅ Successfully connected to Qdrant")

        # Get the total count of points in the collection
        total_count = vector_service.count_total_points()
        print(f"📈 Total vectors stored in collection: {total_count}")

        # Additional stats - let's get collection info as well
        collection_info = vector_service.client.get_collection(settings.QDRANT_COLLECTION_NAME)
        print(f"📦 Collection status: {collection_info.status}")
        print(f"📊 Collection vectors count: {collection_info.vectors_count}")
        print(f"🏷️  Collection segments count: {collection_info.segments_count}")

        if total_count > 0:
            print(f"✅ Success! Found {total_count} embeddings in Qdrant collection.")
            print("✨ Embeddings have been successfully stored!")
        else:
            print("⚠️  No embeddings found in the collection. Either no data has been stored yet, or there might be an issue.")

        return total_count

    except Exception as e:
        print(f"❌ Error checking Qdrant stats: {str(e)}")
        print("Make sure your QDRANT_URL and QDRANT_API_KEY environment variables are set correctly.")
        return None


if __name__ == "__main__":
    main()