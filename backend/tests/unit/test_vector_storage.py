import pytest
from unittest.mock import patch, MagicMock
from src.services.vector_storage import VectorStorageService
from src.models.embedding import VectorEmbedding
from src.models.payload import MetadataPayload
from datetime import datetime


class TestVectorStorageService:
    def test_store_embedding(self, test_settings):
        """Test storing a single embedding."""
        service = VectorStorageService()

        # Create a sample embedding and payload
        embedding = VectorEmbedding(
            vector_id="test-embedding-1",
            vector_data=[0.1, 0.2, 0.3] + [0.0] * 1021,  # 1024-dimensional vector
            chunk_id="test-chunk-1",
            model_used=test_settings.COHERE_MODEL,
            created_at=datetime.now()
        )

        payload = MetadataPayload(
            document_path="docs/test.md",
            section_type="intro",
            chapter_name="test",
            section_heading="Test Section"
        )

        # Mock the Qdrant client
        with patch.object(service, 'client') as mock_client:
            service.store_embedding(embedding, payload)

            # Verify the upsert method was called with correct parameters
            mock_client.upsert.assert_called_once()
            call_args = mock_client.upsert.call_args
            assert call_args[1]['collection_name'] == test_settings.QDRANT_COLLECTION_NAME

            points = call_args[1]['points']
            assert len(points) == 1
            point = points[0]
            assert point.id == "test-embedding-1"
            assert len(point.vector) == 1024
            assert point.payload['document_path'] == "docs/test.md"

    def test_store_embeddings_multiple(self, test_settings):
        """Test storing multiple embeddings."""
        service = VectorStorageService()

        # Create sample embeddings and payloads
        embedding1 = VectorEmbedding(
            vector_id="test-embedding-1",
            vector_data=[0.1, 0.2, 0.3] + [0.0] * 1021,
            chunk_id="test-chunk-1",
            model_used=test_settings.COHERE_MODEL,
            created_at=datetime.now()
        )

        embedding2 = VectorEmbedding(
            vector_id="test-embedding-2",
            vector_data=[0.4, 0.5, 0.6] + [0.0] * 1021,
            chunk_id="test-chunk-2",
            model_used=test_settings.COHERE_MODEL,
            created_at=datetime.now()
        )

        payload1 = MetadataPayload(
            document_path="docs/test1.md",
            section_type="intro",
            chapter_name="test1",
            section_heading="Test Section 1"
        )

        payload2 = MetadataPayload(
            document_path="docs/test2.md",
            section_type="module",
            chapter_name="test2",
            section_heading="Test Section 2"
        )

        embeddings = [embedding1, embedding2]
        payloads = [payload1, payload2]

        # Mock the Qdrant client
        with patch.object(service, 'client') as mock_client:
            service.store_embeddings(embeddings, payloads)

            # Verify the upsert method was called with correct parameters
            mock_client.upsert.assert_called_once()
            call_args = mock_client.upsert.call_args
            assert call_args[1]['collection_name'] == test_settings.QDRANT_COLLECTION_NAME

            points = call_args[1]['points']
            assert len(points) == 2

            # Check first point
            assert points[0].id == "test-embedding-1"
            assert points[0].payload['document_path'] == "docs/test1.md"

            # Check second point
            assert points[1].id == "test-embedding-2"
            assert points[1].payload['document_path'] == "docs/test2.md"

    def test_store_embeddings_mismatched_lengths(self, test_settings):
        """Test that storing embeddings with mismatched payload lengths raises an error."""
        service = VectorStorageService()

        # Create sample embeddings and payloads with different lengths
        embedding = VectorEmbedding(
            vector_id="test-embedding-1",
            vector_data=[0.1, 0.2, 0.3] + [0.0] * 1021,
            chunk_id="test-chunk-1",
            model_used=test_settings.COHERE_MODEL,
            created_at=datetime.now()
        )

        payload = MetadataPayload(
            document_path="docs/test.md",
            section_type="intro",
            chapter_name="test",
            section_heading="Test Section"
        )

        # Should raise ValueError when lengths don't match
        with pytest.raises(ValueError, match="Number of embeddings must match number of payloads"):
            service.store_embeddings([embedding], [])  # Different lengths

    def test_ensure_collection_exists(self, test_settings):
        """Test that collection is created if it doesn't exist."""
        service = VectorStorageService()

        # Mock the Qdrant client to simulate collection not existing
        with patch.object(service, 'client') as mock_client:
            # Simulate that the collection doesn't exist initially
            mock_client.get_collection.side_effect = [Exception(), None]  # First call fails, second succeeds
            mock_client.create_collection = MagicMock()

            # This should trigger collection creation
            service._ensure_collection_exists()

            # Verify create_collection was called
            mock_client.create_collection.assert_called_once()

    def test_validate_payload_validates_required_fields(self, test_settings):
        """Test that payload validation works correctly."""
        service = VectorStorageService()

        # Test valid payload
        valid_payload = MetadataPayload(
            document_path="docs/test.md",
            section_type="intro",
            chapter_name="test",
            section_heading="Test Section"
        )

        # Should not raise an exception
        service._validate_payload(valid_payload)

        # Test invalid payloads
        with pytest.raises(ValueError, match="document_path is required"):
            invalid_payload = MetadataPayload(
                document_path="",
                section_type="intro",
                chapter_name="test",
                section_heading="Test Section"
            )
            service._validate_payload(invalid_payload)

        with pytest.raises(ValueError, match="section_type must be one of"):
            invalid_payload = MetadataPayload(
                document_path="docs/test.md",
                section_type="invalid",
                chapter_name="test",
                section_heading="Test Section"
            )
            service._validate_payload(invalid_payload)

        with pytest.raises(ValueError, match="chapter_name is required"):
            invalid_payload = MetadataPayload(
                document_path="docs/test.md",
                section_type="intro",
                chapter_name="",
                section_heading="Test Section"
            )
            service._validate_payload(invalid_payload)

        with pytest.raises(ValueError, match="section_heading is required"):
            invalid_payload = MetadataPayload(
                document_path="docs/test.md",
                section_type="intro",
                chapter_name="test",
                section_heading=""
            )
            service._validate_payload(invalid_payload)

    def test_store_embedding_validates_payload(self, test_settings):
        """Test that storing an embedding validates the payload."""
        service = VectorStorageService()

        # Create a valid embedding
        embedding = VectorEmbedding(
            vector_id="test-embedding-1",
            vector_data=[0.1, 0.2, 0.3] + [0.0] * 1021,
            chunk_id="test-chunk-1",
            model_used=test_settings.COHERE_MODEL,
            created_at=datetime.now()
        )

        # Valid payload should work
        valid_payload = MetadataPayload(
            document_path="docs/test.md",
            section_type="intro",
            chapter_name="test",
            section_heading="Test Section"
        )

        with patch.object(service, 'client') as mock_client:
            service.store_embedding(embedding, valid_payload)
            mock_client.upsert.assert_called_once()

        # Invalid payload should raise an error
        invalid_payload = MetadataPayload(
            document_path="",  # Invalid - empty
            section_type="intro",
            chapter_name="test",
            section_heading="Test Section"
        )

        with patch.object(service, 'client'):
            with pytest.raises(ValueError, match="document_path is required"):
                service.store_embedding(embedding, invalid_payload)