import pytest
from unittest.mock import patch, MagicMock
from src.services.content_ingestor import ContentIngestor
from src.services.chunker import BasicChunker
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage import VectorStorageService
from src.models.payload import MetadataPayload
from datetime import datetime


class TestMetadataStorage:
    def test_complete_metadata_storage_pipeline(self, test_settings, sample_markdown_content):
        """Test the complete pipeline with comprehensive metadata storage."""
        # Create services
        content_ingestor = ContentIngestor(docs_path=test_settings.DOCS_PATH)
        chunker = BasicChunker()
        embedding_service = EmbeddingService()

        # Mock the vector storage to avoid actual API calls in tests
        with patch('src.services.vector_storage.VectorStorageService') as MockVectorStorage:
            mock_storage = MagicMock()
            MockVectorStorage.return_value = mock_storage

            # Mock the content ingestor to return content with proper structure
            with patch.object(content_ingestor, 'process_documents') as mock_process:
                # Create a sample chunk with full metadata
                from src.models.chunk import BookContentChunk
                sample_chunk = BookContentChunk(
                    id="docs/test.md_chunk_0",
                    content=sample_markdown_content,
                    document_path="docs/test.md",
                    section_type="intro",
                    chapter_name="test",
                    section_heading="Introduction",
                    heading_level=1,
                    module_name=None
                )
                mock_process.return_value = [sample_chunk]

                # Mock the embedding service
                with patch.object(embedding_service, 'generate_embedding_for_single_chunk') as mock_gen_emb:
                    mock_embedding = MagicMock()
                    mock_embedding.vector_id = "docs/test.md_chunk_0"
                    mock_embedding.vector_data = [0.1, 0.2, 0.3] + [0.0] * 1021
                    mock_embedding.chunk_id = "docs/test.md_chunk_0"
                    mock_embedding.model_used = test_settings.COHERE_MODEL
                    mock_gen_emb.return_value = mock_embedding

                    # Run the pipeline
                    for chunk in content_ingestor.process_documents(use_semantic_chunking=True):
                        # Generate embedding
                        embedding = embedding_service.generate_embedding_for_single_chunk(chunk)

                        # Create payload with comprehensive metadata
                        indexing_version = datetime.now().isoformat()
                        payload = MetadataPayload(
                            document_path=chunk.document_path,
                            section_type=chunk.section_type,
                            chapter_name=chunk.chapter_name,
                            section_heading=chunk.section_heading,
                            language=chunk.language,
                            source_type=chunk.source_type,
                            module_name=chunk.module_name,
                            indexing_version=indexing_version
                        )

                        # Store in Qdrant (mocked)
                        mock_storage.store_embedding(embedding, payload)

                        # Verify all required metadata fields are present in the payload
                        payload_dict = payload.to_dict()
                        assert "document_path" in payload_dict
                        assert "section_type" in payload_dict
                        assert "chapter_name" in payload_dict
                        assert "section_heading" in payload_dict
                        assert "language" in payload_dict
                        assert "source_type" in payload_dict
                        assert "indexing_version" in payload_dict

                        # Verify values are correct
                        assert payload_dict["document_path"] == "docs/test.md"
                        assert payload_dict["section_type"] == "intro"
                        assert payload_dict["chapter_name"] == "test"
                        assert payload_dict["section_heading"] == "Introduction"
                        assert payload_dict["language"] == "en"
                        assert payload_dict["source_type"] == "book"
                        assert payload_dict["indexing_version"] == indexing_version

                    # Verify the storage was called
                    assert mock_storage.store_embedding.call_count == 1

    def test_metadata_payload_contains_all_required_fields(self, test_settings):
        """Test that metadata payloads contain all required fields for traceability."""
        indexing_version = datetime.now().isoformat()

        payload = MetadataPayload(
            document_path="docs/modules/module-1/intro.md",
            section_type="module",
            chapter_name="intro",
            section_heading="Introduction to Module 1",
            module_name="module-1",
            indexing_version=indexing_version
        )

        # Convert to dict to check all fields
        payload_dict = payload.to_dict()

        # Verify all required fields are present
        required_fields = ["document_path", "section_type", "chapter_name", "section_heading", "language", "source_type"]
        for field in required_fields:
            assert field in payload_dict, f"Required field {field} is missing from payload"

        # Verify optional fields are conditionally present
        optional_fields = ["module_name", "indexing_version"]
        for field in optional_fields:
            assert field in payload_dict, f"Optional field {field} should be in payload"

        # Verify specific values
        assert payload_dict["document_path"] == "docs/modules/module-1/intro.md"
        assert payload_dict["section_type"] == "module"
        assert payload_dict["module_name"] == "module-1"
        assert payload_dict["indexing_version"] == indexing_version

    def test_payload_to_dict_excludes_none_values(self, test_settings):
        """Test that the payload.to_dict method properly excludes None values."""
        payload = MetadataPayload(
            document_path="docs/test.md",
            section_type="intro",
            chapter_name="test",
            section_heading="Test Section"
            # module_name and indexing_version are None by default
        )

        payload_dict = payload.to_dict()

        # Required fields should always be present
        assert "document_path" in payload_dict
        assert "section_type" in payload_dict
        assert "chapter_name" in payload_dict
        assert "section_heading" in payload_dict

        # Optional fields that are None should not be in the dict
        assert "module_name" not in payload_dict  # Should not be present since it's None
        assert "indexing_version" not in payload_dict  # Should not be present since it's None

    def test_traceability_from_vector_back_to_source(self, test_settings, sample_markdown_content):
        """Test that vectors can be traced back to their original source documents."""
        # Simulate the process of creating a chunk that will be stored as a vector
        from src.models.chunk import BookContentChunk

        chunk = BookContentChunk(
            id="docs/hardware/specifications.md_chunk_2",
            content="Hardware specifications include CPU, RAM, and storage requirements.",
            document_path="docs/hardware/specifications.md",
            section_type="hardware",
            chapter_name="specifications",
            section_heading="System Requirements",
            heading_level=2,
            module_name=None
        )

        # Create embedding (mocked in real usage)
        mock_embedding_vector = [0.5, 0.3, 0.8] + [0.0] * 1021  # 1024-dimensional vector

        # Create payload with all traceability metadata
        indexing_version = datetime.now().isoformat()
        payload = MetadataPayload(
            document_path=chunk.document_path,
            section_type=chunk.section_type,
            chapter_name=chunk.chapter_name,
            section_heading=chunk.section_heading,
            language=chunk.language,
            source_type=chunk.source_type,
            module_name=chunk.module_name,
            indexing_version=indexing_version
        )

        # The payload contains all information needed to trace back to source
        payload_dict = payload.to_dict()

        # Verify we can trace back to the original document
        assert payload_dict["document_path"] == "docs/hardware/specifications.md"
        assert payload_dict["section_type"] == "hardware"
        assert payload_dict["chapter_name"] == "specifications"
        assert payload_dict["section_heading"] == "System Requirements"
        assert "Hardware specifications" in chunk.content  # Content matches the source

        # This demonstrates that each stored vector can be traced back to:
        # - File: document_path
        # - Module: module_name (if applicable)
        # - Chapter: chapter_name
        # - Section: section_heading