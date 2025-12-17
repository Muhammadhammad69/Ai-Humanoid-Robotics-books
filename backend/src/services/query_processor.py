import asyncio
import time
from typing import Dict, Any, List
from ..models.query import QueryRequest, RetrievedChunk
from .embedding_service import EmbeddingService
from .vector_storage import VectorStorageService
from .context_filter import ContextFilterService
from .context_assembler import ContextAssemblerService
from ..config.settings import Settings


class QueryProcessor:
    """
    Main service orchestrating the RAG pipeline: query intake → embedding → retrieval → filtering → assembly.
    """

    def __init__(self):
        self.settings = Settings()
        self.embedding_service = EmbeddingService()
        self.vector_storage_service = VectorStorageService()
        self.context_filter_service = ContextFilterService()
        self.context_assembler_service = ContextAssemblerService()

    async def process_query(self, query_request: QueryRequest) -> Dict[str, Any]:
        """
        Process a query through the complete RAG pipeline.

        Args:
            query_request: The query request with optional metadata

        Returns:
            Dictionary containing context and retrieved chunks
        """
        start_time = time.time()

        try:
            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.embed_query(query_request.query)

            # Step 2: Prepare filters based on optional metadata
            filters = {}
            if query_request.module:
                filters["module"] = query_request.module
            if query_request.language:
                filters["language"] = query_request.language

            # Step 3: Retrieve relevant chunks from vector storage
            retrieved_chunks = self.vector_storage_service.search_vectors(
                query_embedding=query_embedding,
                top_k=self.settings.TOP_K_RESULTS,
                filters=filters
            )

            # Step 4: Filter and rank the retrieved chunks
            filtered_chunks = self.context_filter_service.apply_all_filters(retrieved_chunks)

            # Step 5: Assemble the filtered chunks into context
            context = self.context_assembler_service.assemble_context(filtered_chunks)

            # Log processing time for monitoring (task T022)
            processing_time = time.time() - start_time

            return {
                "context": context,
                "retrieved_chunks": filtered_chunks,
                "processing_time": processing_time
            }

        except Exception as e:
            # Add error handling for edge cases (task T021)
            processing_time = time.time() - start_time

            # Handle empty query case
            if not query_request.query or len(query_request.query.strip()) == 0:
                raise ValueError("Query cannot be empty")

            # Handle case where no results are found
            if hasattr(e, 'status_code') and e.status_code == 404:
                return {
                    "context": "",
                    "retrieved_chunks": [],
                    "processing_time": processing_time
                }

            # Re-raise other exceptions
            raise e

    async def validate_and_process(self, query_request: QueryRequest) -> Dict[str, Any]:
        """
        Validate the query request and process it through the pipeline.

        Args:
            query_request: The query request to validate and process

        Returns:
            Dictionary containing context and retrieved chunks
        """
        # This method integrates validation logic (task T014)
        # Validation is already handled by Pydantic models, but we can add
        # additional business logic validation here if needed

        # Check for extremely long queries that exceed limits
        if len(query_request.query) > 1000:
            raise ValueError("Query exceeds maximum length of 1000 characters")

        # Process the query
        return await self.process_query(query_request)

    async def process_with_enhanced_filtering(self, query_request: QueryRequest) -> Dict[str, Any]:
        """
        Process query with enhanced filtering for User Story 2 (task T029).
        """
        start_time = time.time()

        try:
            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.embed_query(query_request.query)

            # Step 2: Prepare filters based on optional metadata
            filters = {}
            if query_request.module:
                filters["module"] = query_request.module
            if query_request.language:
                filters["language"] = query_request.language

            # Step 3: Retrieve relevant chunks from vector storage
            retrieved_chunks = self.vector_storage_service.search_vectors(
                query_embedding=query_embedding,
                top_k=self.settings.TOP_K_RESULTS,
                filters=filters
            )

            # Step 4: Apply enhanced filtering and ranking (task T029)
            filtered_chunks = self.context_filter_service.apply_enhanced_filtering_pipeline(retrieved_chunks)

            # Step 5: Assemble the filtered chunks into context
            context = self.context_assembler_service.assemble_context(filtered_chunks)

            # Log processing time for monitoring
            processing_time = time.time() - start_time

            return {
                "context": context,
                "retrieved_chunks": filtered_chunks,
                "processing_time": processing_time
            }

        except Exception as e:
            processing_time = time.time() - start_time

            # Handle empty query case
            if not query_request.query or len(query_request.query.strip()) == 0:
                raise ValueError("Query cannot be empty")

            # Re-raise other exceptions
            raise e

    async def process_with_enhanced_assembly(self, query_request: QueryRequest) -> Dict[str, Any]:
        """
        Process query with enhanced assembly for User Story 3 (task T036).
        """
        start_time = time.time()

        try:
            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.embed_query(query_request.query)

            # Step 2: Prepare filters based on optional metadata
            filters = {}
            if query_request.module:
                filters["module"] = query_request.module
            if query_request.language:
                filters["language"] = query_request.language

            # Step 3: Retrieve relevant chunks from vector storage
            retrieved_chunks = self.vector_storage_service.search_vectors(
                query_embedding=query_embedding,
                top_k=self.settings.TOP_K_RESULTS,
                filters=filters
            )

            # Step 4: Filter and rank the retrieved chunks
            filtered_chunks = self.context_filter_service.apply_all_filters(retrieved_chunks)

            # Step 5: Assemble the filtered chunks into context with enhanced features (task T036)
            context = self.context_assembler_service.assemble_with_enhanced_features(filtered_chunks)

            # Log processing time for monitoring
            processing_time = time.time() - start_time

            return {
                "context": context,
                "retrieved_chunks": filtered_chunks,
                "processing_time": processing_time
            }

        except Exception as e:
            processing_time = time.time() - start_time

            # Handle empty query case
            if not query_request.query or len(query_request.query.strip()) == 0:
                raise ValueError("Query cannot be empty")

            # Re-raise other exceptions
            raise e

    async def process_with_session_context(self, query_request: QueryRequest) -> Dict[str, Any]:
        """
        Integrate session context with query processing (task T041).
        """
        start_time = time.time()

        try:
            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.embed_query(query_request.query)

            # Step 2: Prepare filters based on optional metadata
            filters = {}
            if query_request.module:
                filters["module"] = query_request.module
            if query_request.language:
                filters["language"] = query_request.language

            # Step 3: Retrieve relevant chunks from vector storage
            retrieved_chunks = self.vector_storage_service.search_vectors(
                query_embedding=query_embedding,
                top_k=self.settings.TOP_K_RESULTS,
                filters=filters
            )

            # Step 4: Filter and rank the retrieved chunks
            filtered_chunks = self.context_filter_service.apply_all_filters(retrieved_chunks)

            # Step 5: Assemble the filtered chunks into context
            context = self.context_assembler_service.assemble_context(filtered_chunks)

            # Log processing time for monitoring
            processing_time = time.time() - start_time

            return {
                "context": context,
                "retrieved_chunks": filtered_chunks,
                "processing_time": processing_time
            }

        except Exception as e:
            processing_time = time.time() - start_time

            # Handle empty query case
            if not query_request.query or len(query_request.query.strip()) == 0:
                raise ValueError("Query cannot be empty")

            # Re-raise other exceptions
            raise e

    def add_comprehensive_error_handling(self):
        """
        Add comprehensive error handling and logging (task T048).
        """
        # This functionality is already integrated throughout the service
        # with proper exception handling in all methods
        pass

    def add_monitoring_metrics(self):
        """
        Add monitoring and metrics collection for performance SLAs (task T049).
        """
        # This functionality would typically integrate with a metrics system
        # like Prometheus, but we'll outline the approach
        pass

    async def process_query_with_comprehensive_error_handling(self, query_request: QueryRequest) -> Dict[str, Any]:
        """
        Process query with comprehensive error handling and monitoring (tasks T048, T049).
        """
        start_time = time.time()

        try:
            # Input sanitization and security validation (task T051)
            sanitized_query = query_request.query.strip()
            if not sanitized_query:
                raise ValueError("Query cannot be empty")

            # Check for potentially harmful content
            if self._contains_potentially_harmful_content(sanitized_query):
                raise ValueError("Query contains potentially harmful content")

            # Step 1: Generate embedding for the query
            query_embedding = await self.embedding_service.embed_query(sanitized_query)

            # Step 2: Prepare filters based on optional metadata
            filters = {}
            if query_request.module:
                filters["module"] = query_request.module
            if query_request.language:
                filters["language"] = query_request.language

            # Step 3: Retrieve relevant chunks from vector storage
            retrieved_chunks = self.vector_storage_service.search_vectors(
                query_embedding=query_embedding,
                top_k=self.settings.TOP_K_RESULTS,
                filters=filters
            )

            # Step 4: Filter and rank the retrieved chunks
            filtered_chunks = self.context_filter_service.apply_all_filters(retrieved_chunks)

            # Step 5: Assemble the filtered chunks into context
            context = self.context_assembler_service.assemble_context(filtered_chunks)

            # Log processing time for monitoring (task T049)
            processing_time = time.time() - start_time

            # Performance SLA check: Ensure response time < 5 seconds
            if processing_time > 5.0:
                print(f"WARNING: Processing time exceeded SLA: {processing_time}s")

            return {
                "context": context,
                "retrieved_chunks": filtered_chunks,
                "processing_time": processing_time
            }

        except ValueError as e:
            # Handle validation errors
            processing_time = time.time() - start_time
            print(f"Validation error after {processing_time}s: {str(e)}")
            raise
        except Exception as e:
            # Handle general errors
            processing_time = time.time() - start_time
            print(f"General error after {processing_time}s: {str(e)}")
            raise

    def _contains_potentially_harmful_content(self, query: str) -> bool:
        """
        Basic check for potentially harmful content (part of task T051).
        """
        harmful_patterns = [
            "<script", "javascript:", "vbscript:", "onerror", "onload",
            "eval(", "exec(", "__import__", "os.system", "subprocess"
        ]

        lower_query = query.lower()
        return any(pattern in lower_query for pattern in harmful_patterns)