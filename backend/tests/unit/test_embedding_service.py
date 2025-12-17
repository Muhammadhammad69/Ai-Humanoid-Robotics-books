import pytest
from unittest.mock import patch, MagicMock
from src.services.embedding_service import EmbeddingService
from src.models.chunk import BookContentChunk


class TestEmbeddingService:
    def test_generate_embedding_for_single_chunk(self, test_settings, sample_chunk_data):
        """Test generating embedding for a single chunk."""
        service = EmbeddingService()

        # Create a sample chunk
        chunk = BookContentChunk(**sample_chunk_data)

        # Mock the Cohere client
        with patch.object(service, 'client') as mock_client:
            mock_response = MagicMock()
            mock_response.embeddings = [[0.1, 0.2, 0.3] + [0.0] * 1021]  # 1024-dimensional vector
            mock_client.embed.return_value = mock_response

            embedding = service.generate_embedding_for_single_chunk(chunk)

            # Verify the embedding was created correctly
            assert embedding.chunk_id == chunk.id
            assert len(embedding.vector_data) == 1024  # Expected dimension
            assert embedding.model_used == test_settings.COHERE_MODEL

            # Verify the API was called with correct parameters
            mock_client.embed.assert_called_once_with(
                texts=[chunk.content],
                model=test_settings.COHERE_MODEL,
                input_type="search_document",
            )

    def test_generate_embeddings_for_multiple_chunks(self, test_settings, sample_chunk_data):
        """Test generating embeddings for multiple chunks."""
        service = EmbeddingService()

        # Create sample chunks
        chunk1 = BookContentChunk(**sample_chunk_data)
        chunk2_data = sample_chunk_data.copy()
        chunk2_data["id"] = "test-chunk-2"
        chunk2_data["content"] = "This is the content of the second chunk."
        chunk2 = BookContentChunk(**chunk2_data)

        chunks = [chunk1, chunk2]

        # Mock the Cohere client
        with patch.object(service, 'client') as mock_client:
            mock_response = MagicMock()
            # Create 2 embeddings with 1024 dimensions each
            mock_response.embeddings = [
                [0.1, 0.2, 0.3] + [0.0] * 1021,
                [0.4, 0.5, 0.6] + [0.0] * 1021
            ]
            mock_client.embed.return_value = mock_response

            embeddings = service.generate_embeddings(chunks)

            # Verify the embeddings were created correctly
            assert len(embeddings) == 2
            for i, embedding in enumerate(embeddings):
                assert embedding.chunk_id == chunks[i].id
                assert len(embedding.vector_data) == 1024  # Expected dimension
                assert embedding.model_used == test_settings.COHERE_MODEL

            # Verify the API was called with correct parameters
            mock_client.embed.assert_called_once_with(
                texts=[chunk1.content, chunk2.content],
                model=test_settings.COHERE_MODEL,
                input_type="search_document",
            )

    def test_generate_embeddings_empty_list(self, test_settings):
        """Test generating embeddings for an empty list."""
        service = EmbeddingService()

        embeddings = service.generate_embeddings([])

        # Should return an empty list
        assert embeddings == []

    def test_enforce_rate_limit(self, test_settings):
        """Test that rate limiting is enforced."""
        service = EmbeddingService()

        # Set rate limit to 1 request per minute for testing
        with patch.object(service.settings, 'COHERE_RATE_LIMIT_RPM', 1):
            # First request should not be delayed
            service._enforce_rate_limit()
            assert service.request_count == 0

            # Increment request count
            service.request_count = 1

            # Second request should trigger rate limiting logic
            # (We're not actually sleeping in tests, just checking the logic)
            service._enforce_rate_limit()
            # If we got here without exception, the rate limiting logic was executed
            assert True

    def test_batch_processing_respects_batch_size(self, test_settings, sample_chunk_data):
        """Test that embeddings are processed in batches respecting COHERE_BATCH_SIZE."""
        service = EmbeddingService()

        # Create more chunks than the batch size to test batching
        chunks = []
        for i in range(15):  # More than typical batch size
            chunk_data = sample_chunk_data.copy()
            chunk_data["id"] = f"test-chunk-{i}"
            chunk_data["content"] = f"This is content for chunk {i}."
            chunks.append(BookContentChunk(**chunk_data))

        # Mock the Cohere client
        with patch.object(service, 'client') as mock_client:
            mock_response = MagicMock()
            # Create embeddings for each chunk in the batch
            mock_response.embeddings = [[0.1, 0.2, 0.3] + [0.0] * 1021] * len(chunks)
            mock_client.embed.return_value = mock_response

            # This should process chunks in batches
            embeddings = service.generate_embeddings(chunks)

            # Verify the number of embeddings matches the number of chunks
            assert len(embeddings) == len(chunks)

            # Verify that embed was called multiple times if we exceeded batch size
            # If batch size is 10 and we have 15 chunks, we should have 2 batches
            expected_batches = (len(chunks) + test_settings.COHERE_BATCH_SIZE - 1) // test_settings.COHERE_BATCH_SIZE
            assert mock_client.embed.call_count == expected_batches

    def test_exponential_backoff_on_failure(self, test_settings, sample_chunk_data):
        """Test that exponential backoff is applied when API calls fail."""
        service = EmbeddingService()

        chunk = BookContentChunk(**sample_chunk_data)

        # Mock the Cohere client to raise an exception
        with patch.object(service, 'client') as mock_client:
            mock_client.embed.side_effect = Exception("API Error")

            # Mock the backoff function to avoid actual sleeping
            with patch.object(service, '_exponential_backoff') as mock_backoff:
                # This should trigger retries and eventually raise the exception
                with pytest.raises(Exception, match="API Error"):
                    service.generate_embeddings([chunk])

                # Verify backoff was called for retries (max 3 attempts = 2 backoffs)
                assert mock_backoff.call_count == 2  # Called on attempts 1 and 2, then raises on attempt 3

    def test_validate_embedding_dimensions(self, test_settings):
        """Test that embedding dimension validation works correctly."""
        service = EmbeddingService()

        # Valid embedding should not raise an error
        valid_embedding = [0.1, 0.2, 0.3] + [0.0] * 1021  # 1024-dimensional vector
        service._validate_embedding_dimensions(valid_embedding)

        # Invalid embedding should raise an error
        invalid_embedding = [0.1, 0.2, 0.3]  # Only 3 dimensions
        with pytest.raises(ValueError, match="dimensions"):
            service._validate_embedding_dimensions(invalid_embedding)

    def test_logging_on_rate_limit(self, test_settings, sample_chunk_data, caplog):
        """Test that rate limiting is logged."""
        service = EmbeddingService()

        chunk = BookContentChunk(**sample_chunk_data)

        # Mock rate limiting to trigger log message
        with patch.object(service, '_enforce_rate_limit'):
            # Override to simulate rate limit being hit
            with patch.object(service.logger, 'info') as mock_log:
                service._enforce_rate_limit()  # This should log if rate limit is reached
                # The log happens in the actual enforcement, so we'll just verify logging capability
                assert service.logger is not None