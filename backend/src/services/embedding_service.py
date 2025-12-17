import time
import random
import uuid
from typing import List, Optional
from cohere import Client
from src.config.settings import Settings
from src.models.chunk import BookContentChunk
from src.models.embedding import VectorEmbedding
from datetime import datetime
import logging


class EmbeddingService:
    """
    Service to generate embeddings using Cohere API.
    """

    def __init__(self):
        self.settings = Settings()
        self.client = Client(api_key=self.settings.COHERE_API_KEY)
        self.last_request_time = 0
        self.request_count = 0
        self.rate_limit_window_start = time.time()
        self.logger = logging.getLogger(__name__)

    def _enforce_rate_limit(self):
        """Enforce rate limits based on COHERE_RATE_LIMIT_RPM."""
        current_time = time.time()

        # Reset the counter if we're in a new minute window
        if current_time - self.rate_limit_window_start >= 60:
            self.rate_limit_window_start = current_time
            self.request_count = 0

        # If we've hit the rate limit, wait until the window resets
        if self.request_count >= self.settings.COHERE_RATE_LIMIT_RPM:
            sleep_time = 60 - (current_time - self.rate_limit_window_start)
            if sleep_time > 0:
                self.logger.info(f"Rate limit reached. Sleeping for {sleep_time:.2f} seconds.")
                time.sleep(sleep_time)
                self.rate_limit_window_start = time.time()
                self.request_count = 0

    def _exponential_backoff(self, attempt: int, max_delay: float = 60.0):
        """
        Implement exponential backoff with jitter for API errors.

        Args:
            attempt: Current attempt number (0-indexed)
            max_delay: Maximum delay in seconds
        """
        # Calculate base delay with exponential backoff
        base_delay = min(max_delay, (2 ** attempt))
        # Add jitter to prevent thundering herd
        jitter = random.uniform(0, 0.1 * base_delay)
        delay = base_delay + jitter

        self.logger.warning(f"Attempt {attempt + 1}: Waiting {delay:.2f}s before retrying...")
        time.sleep(delay)

    def _validate_embedding_dimensions(self, embedding_vector: List[float]) -> None:
        """
        Validate that the embedding vector has the expected dimensions.

        Args:
            embedding_vector: The embedding vector to validate
        """
        if len(embedding_vector) != self.settings.COHERE_OUTPUT_DIM:
            raise ValueError(
                f"Embedding vector has {len(embedding_vector)} dimensions, "
                f"expected {self.settings.COHERE_OUTPUT_DIM}"
            )

    def generate_embeddings(self, chunks: List[BookContentChunk]) -> List[VectorEmbedding]:
        """
        Generate embeddings for a list of chunks with proper batching and rate limiting.

        Args:
            chunks: List of BookContentChunk objects

        Returns:
            List of VectorEmbedding objects
        """
        if not chunks:
            self.logger.info("Empty chunk list provided to generate_embeddings")
            return []

        # Validate input chunks
        for i, chunk in enumerate(chunks):
            if not chunk.content or not chunk.content.strip():
                self.logger.warning(f"Chunk {i} has empty content, skipping...")
                continue

        embeddings = []

        # Process chunks in batches respecting the COHERE_BATCH_SIZE
        for i in range(0, len(chunks), self.settings.COHERE_BATCH_SIZE):
            batch = chunks[i:i + self.settings.COHERE_BATCH_SIZE]

            # Apply rate limiting before each batch request
            self._enforce_rate_limit()

            # Prepare the text for embedding
            texts = [chunk.content for chunk in batch]

            max_retries = 3
            for attempt in range(max_retries):
                try:
                    # Call Cohere API to generate embeddings
                    self.logger.debug(f"Sending batch of {len(texts)} texts to Cohere API")
                    response = self.client.embed(
                        texts=texts,
                        model=self.settings.COHERE_MODEL,
                        input_type="search_document",  # Using search_document for book content
                    )

                    # Increment request count
                    self.request_count += 1
                    self.logger.debug(f"Received embeddings for batch, request count: {self.request_count}")

                    # Create VectorEmbedding objects from the response
                    for j, embedding_vector in enumerate(response.embeddings):
                        self._validate_embedding_dimensions(embedding_vector)

                        embedding = VectorEmbedding(
                            vector_id=str(uuid.uuid4()),  # Use a UUID for the embedding ID
                            vector_data=embedding_vector,
                            chunk_id=batch[j].id,
                            model_used=self.settings.COHERE_MODEL,
                            created_at=datetime.now()
                        )
                        embeddings.append(embedding)

                    # Successfully processed this batch, break retry loop
                    self.logger.info(f"Successfully processed batch {i//self.settings.COHERE_BATCH_SIZE + 1} with {len(batch)} chunks")
                    break

                except Exception as e:
                    self.logger.error(f"Error generating embeddings for batch {i//self.settings.COHERE_BATCH_SIZE + 1} (attempt {attempt + 1}): {str(e)}")

                    if attempt == max_retries - 1:
                        # Last attempt, re-raise the error with additional context
                        self.logger.error(f"All retry attempts failed for batch {i//self.settings.COHERE_BATCH_SIZE + 1}. Last error: {str(e)}")
                        raise e

                    # Exponential backoff before retry
                    self._exponential_backoff(attempt)

        self.logger.info(f"Successfully generated embeddings for {len(chunks)} chunks, resulting in {len(embeddings)} vectors")
        return embeddings

    def generate_embedding_for_single_chunk(self, chunk: BookContentChunk) -> VectorEmbedding:
        """
        Generate embedding for a single chunk.

        Args:
            chunk: A BookContentChunk object

        Returns:
            A VectorEmbedding object
        """
        return self.generate_embeddings([chunk])[0]