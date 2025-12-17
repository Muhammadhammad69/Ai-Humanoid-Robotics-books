#!/usr/bin/env python3
"""
Command-line interface for the book content indexer.
"""

import argparse
import logging
import sys
from pathlib import Path
from datetime import datetime

# Add the backend directory to the path so we can import modules
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.services.content_ingestor import ContentIngestor
from src.services.chunker import BasicChunker
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService
from src.config.settings import Settings
from src.models.payload import MetadataPayload


def setup_logging():
    """Set up logging for the indexer."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)


def main():
    logger = setup_logging()

    parser = argparse.ArgumentParser(description="Book Content Indexer")
    parser.add_argument(
        "--docs-path",
        type=str,
        default=Settings.DOCS_PATH,
        help="Path to the docs directory (default: from environment)"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without storing vectors in Qdrant"
    )
    parser.add_argument(
        "--reindex",
        action="store_true",
        help="Perform a full re-index, replacing all existing vectors"
    )
    parser.add_argument(
        "--incremental",
        action="store_true",
        help="Perform incremental update for changed documents only"
    )

    args = parser.parse_args()

    print("Starting book content indexing process...")

    # Initialize services
    content_ingestor = ContentIngestor(docs_path=args.docs_path)
    chunker = BasicChunker()
    embedding_service = EmbeddingService()

    if not args.dry_run:
        vector_storage = VectorStorageService()

    # Handle re-indexing if requested
    if args.reindex and not args.dry_run:
        logger.info("Performing full re-index - clearing existing vectors...")
        total_points_before = vector_storage.count_total_points()
        vector_storage.clear_collection()
        logger.info(f"Cleared {total_points_before} existing vectors from collection")

    # Process documents with semantic chunking
    chunk_count = 0
    for chunk in content_ingestor.process_documents(use_semantic_chunking=True):
        logger.info(f"Processing: {chunk.document_path} - Chunk {chunk_count} - Heading: {chunk.section_heading}")

        # Generate embedding
        logger.info(f"  Generating embedding for chunk...")
        try:
            embedding = embedding_service.generate_embedding_for_single_chunk(chunk)
        except Exception as e:
            logger.error(f"  Failed to generate embedding for chunk {chunk.id}: {str(e)}")
            continue

        if not args.dry_run:
            # Create payload for storage
            indexing_version = datetime.now().isoformat()  # Use timestamp as indexing version
            payload = MetadataPayload(
                document_path=chunk.document_path,
                section_type=chunk.section_type,
                chapter_name=chunk.chapter_name,
                section_heading=chunk.section_heading,
                content=chunk.content,  # Include the actual chunk text
                language=chunk.language,
                source_type=chunk.source_type,
                module_name=chunk.module_name,
                indexing_version=indexing_version
            )

            # Store in Qdrant
            logger.info(f"  Storing vector in Qdrant...")
            try:
                vector_storage.store_embedding(embedding, payload)
            except Exception as e:
                logger.error(f"  Failed to store embedding for chunk {chunk.id}: {str(e)}")
                continue

        chunk_count += 1
        logger.info(f"  Completed chunk {chunk_count}")

    print(f"Indexing completed! Processed {chunk_count} chunks.")

    if not args.dry_run:
        total_points_after = vector_storage.count_total_points()
        logger.info(f"Total vectors in collection after indexing: {total_points_after}")


if __name__ == "__main__":
    main()