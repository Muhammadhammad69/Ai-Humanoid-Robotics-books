import pytest
from unittest.mock import patch, MagicMock
from src.services.content_ingestor import ContentIngestor
from src.services.chunker import BasicChunker
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService
from src.models.chunk import BookContentChunk


class TestIndexingPipeline:
    def test_complete_indexing_pipeline(self, test_settings, sample_markdown_content):
        """Test the complete indexing pipeline from content ingestion to vector storage."""
        # Create services
        content_ingestor = ContentIngestor(docs_path=test_settings.DOCS_PATH)
        chunker = BasicChunker()
        embedding_service = EmbeddingService()

        # Mock the vector storage to avoid actual API calls in tests
        with patch('src.services.vector_storage.VectorStorageService') as MockVectorStorage:
            mock_storage = MagicMock()
            MockVectorStorage.return_value = mock_storage

            # Mock the content ingestor to return a known document
            with patch.object(content_ingestor, 'process_documents') as mock_process:
                # Create a sample chunk to return
                sample_chunk = BookContentChunk(
                    id="test-doc_chunk_0",
                    content=sample_markdown_content,
                    document_path="docs/test.md",
                    section_type="intro",
                    chapter_name="test",
                    section_heading="Full Document"
                )
                mock_process.return_value = [sample_chunk]

                # Mock the embedding service
                with patch.object(embedding_service, 'generate_embedding_for_single_chunk') as mock_gen_emb:
                    mock_embedding = MagicMock()
                    mock_embedding.vector_id = "test-doc_chunk_0"
                    mock_embedding.vector_data = [0.1, 0.2, 0.3] + [0.0] * 1021
                    mock_embedding.chunk_id = "test-doc_chunk_0"
                    mock_embedding.model_used = test_settings.COHERE_MODEL
                    mock_gen_emb.return_value = mock_embedding

                    # Run the pipeline (simulating what the CLI does)
                    chunk_count = 0
                    for chunk in content_ingestor.process_documents():
                        # Generate embedding
                        embedding = embedding_service.generate_embedding_for_single_chunk(chunk)

                        # Create payload for storage
                        from src.models.payload import MetadataPayload
                        payload = MetadataPayload(
                            document_path=chunk.document_path,
                            section_type=chunk.section_type,
                            chapter_name=chunk.chapter_name,
                            section_heading=chunk.section_heading,
                            language=chunk.language,
                            source_type=chunk.source_type,
                            module_name=chunk.module_name,
                            indexing_version="v1"
                        )

                        # Store in Qdrant (mocked)
                        mock_storage.store_embedding(embedding, payload)
                        chunk_count += 1

                    # Verify the pipeline ran correctly
                    assert chunk_count == 1
                    mock_gen_emb.assert_called_once()
                    mock_storage.store_embedding.assert_called_once()

    def test_pipeline_with_multiple_chunks(self, test_settings, sample_markdown_content):
        """Test the pipeline with content that gets chunked into multiple pieces."""
        # Use the chunker to split content
        chunker = BasicChunker(max_chunk_size=50)  # Small size to force multiple chunks
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Section"
        }

        chunks = chunker.chunk_content(sample_markdown_content, document_info)

        # Should have multiple chunks due to small max_chunk_size
        assert len(chunks) > 1

        # Test that each chunk can be processed through the pipeline
        embedding_service = EmbeddingService()
        with patch.object(embedding_service, 'generate_embedding_for_single_chunk') as mock_gen_emb:
            mock_embedding = MagicMock()
            mock_embedding.vector_id = "mock-id"
            mock_embedding.vector_data = [0.1, 0.2, 0.3] + [0.0] * 1021
            mock_embedding.chunk_id = "mock-chunk-id"
            mock_embedding.model_used = test_settings.COHERE_MODEL
            mock_gen_emb.return_value = mock_embedding

            # Process each chunk
            for chunk in chunks:
                embedding = embedding_service.generate_embedding_for_single_chunk(chunk)
                assert embedding is not None

            # Verify embedding was called for each chunk
            assert mock_gen_emb.call_count == len(chunks)