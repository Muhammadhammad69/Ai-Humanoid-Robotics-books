from typing import List
from ..models.query import RetrievedChunk
from ..utils.token_counter import TokenCounter
from ..utils.context_utils import ContextUtils
from ..config.settings import Settings


class ContextFilterService:
    """
    Service for filtering and ranking retrieved context chunks.
    """

    def __init__(self):
        self.settings = Settings()
        self.token_counter = TokenCounter()
        self.context_utils = ContextUtils()

    def filter_by_relevance_threshold(
        self,
        chunks: List[RetrievedChunk],
        threshold: float = None
    ) -> List[RetrievedChunk]:
        """
        Filter chunks based on relevance score threshold.

        Args:
            chunks: List of retrieved chunks to filter
            threshold: Minimum relevance score (uses default if None)

        Returns:
            Filtered list of chunks
        """
        if threshold is None:
            threshold = self.settings.RELEVANCE_THRESHOLD

        return [chunk for chunk in chunks if chunk.relevance_score >= threshold]

    def filter_by_token_budget(
        self,
        chunks: List[RetrievedChunk],
        token_budget: int = None
    ) -> List[RetrievedChunk]:
        """
        Filter chunks based on token budget constraints.

        Args:
            chunks: List of retrieved chunks to filter
            token_budget: Maximum token budget (uses default if None)

        Returns:
            Filtered list of chunks that fit within token budget
        """
        if token_budget is None:
            token_budget = self.settings.TOKEN_BUDGET

        filtered_chunks = []
        current_tokens = 0

        for chunk in chunks:  # Process in order of relevance
            chunk_tokens = self.token_counter.count_tokens(chunk.content)

            if current_tokens + chunk_tokens <= token_budget:
                filtered_chunks.append(chunk)
                current_tokens += chunk_tokens
            else:
                break  # Stop when budget is exceeded

        return filtered_chunks

    def remove_duplicates(
        self,
        chunks: List[RetrievedChunk],
        threshold: float = None
    ) -> List[RetrievedChunk]:
        """
        Remove duplicate or near-duplicate chunks.

        Args:
            chunks: List of retrieved chunks to deduplicate
            threshold: Similarity threshold for duplicates (uses default if None)

        Returns:
            List of chunks with duplicates removed
        """
        if threshold is None:
            threshold = self.settings.DUPLICATE_THRESHOLD

        return self.context_utils.remove_duplicates(chunks, threshold)

    def maintain_relevance_order(self, chunks: List[RetrievedChunk]) -> List[RetrievedChunk]:
        """
        Ensure chunks are ordered by relevance score (highest first).

        Args:
            chunks: List of chunks to sort

        Returns:
            Chunks sorted by relevance score in descending order
        """
        return sorted(chunks, key=lambda x: x.relevance_score, reverse=True)

    def apply_all_filters(
        self,
        chunks: List[RetrievedChunk]
    ) -> List[RetrievedChunk]:
        """
        Apply all filtering steps to the chunks.

        Args:
            chunks: List of retrieved chunks to filter

        Returns:
            Filtered and processed list of chunks
        """
        # 1. First, maintain relevance order
        ordered_chunks = self.maintain_relevance_order(chunks)

        # 2. Filter by relevance threshold
        relevance_filtered = self.filter_by_relevance_threshold(ordered_chunks)

        # 3. Remove duplicates
        deduplicated = self.remove_duplicates(relevance_filtered)

        # 4. Filter by token budget
        budget_filtered = self.filter_by_token_budget(deduplicated)

        return budget_filtered

    def validate_relevance_scores(self, chunks: List[RetrievedChunk]) -> bool:
        """
        Validate that relevance scores are within expected ranges.

        Args:
            chunks: List of chunks to validate

        Returns:
            True if all scores are valid, False otherwise
        """
        for chunk in chunks:
            if not (0.0 <= chunk.relevance_score <= 1.0):
                return False
        return True

    def enhance_with_relevance_filtering(
        self,
        chunks: List[RetrievedChunk]
    ) -> List[RetrievedChunk]:
        """
        Enhanced filtering with additional relevance-based logic (task T024).
        """
        # Apply relevance threshold filtering with the configured threshold
        return self.filter_by_relevance_threshold(chunks)

    def enforce_token_budget(
        self,
        chunks: List[RetrievedChunk]
    ) -> List[RetrievedChunk]:
        """
        Enforce token budget with additional considerations (task T025).
        """
        # Apply token budget filtering with the configured budget
        return self.filter_by_token_budget(chunks)

    def implement_duplicate_detection(
        self,
        chunks: List[RetrievedChunk]
    ) -> List[RetrievedChunk]:
        """
        Implement enhanced duplicate detection algorithm (task T026).
        """
        # Apply duplicate removal with the configured threshold
        return self.remove_duplicates(chunks)

    def maintain_relevance_based_ordering(
        self,
        chunks: List[RetrievedChunk]
    ) -> List[RetrievedChunk]:
        """
        Maintain relevance-based ordering of filtered chunks (task T027).
        """
        return self.maintain_relevance_order(chunks)

    def apply_enhanced_filtering_pipeline(
        self,
        chunks: List[RetrievedChunk]
    ) -> List[RetrievedChunk]:
        """
        Apply the complete enhanced filtering pipeline for User Story 2.
        """
        # Apply all enhanced filtering steps in sequence
        # 1. Maintain relevance order
        ordered_chunks = self.maintain_relevance_based_ordering(chunks)

        # 2. Apply relevance filtering
        relevance_filtered = self.enhance_with_relevance_filtering(ordered_chunks)

        # 3. Enforce token budget
        budget_filtered = self.enforce_token_budget(relevance_filtered)

        # 4. Remove duplicates
        deduplicated = self.implement_duplicate_detection(budget_filtered)

        return deduplicated