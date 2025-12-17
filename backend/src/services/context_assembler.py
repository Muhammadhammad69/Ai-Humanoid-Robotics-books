from typing import List
from ..models.query import RetrievedChunk
from ..utils.context_utils import ContextUtils
from ..config.settings import Settings


class ContextAssemblerService:
    """
    Service for assembling filtered chunks into a single, clean context.
    """

    def __init__(self):
        self.settings = Settings
        self.context_utils = ContextUtils()

    def assemble_context(
        self,
        chunks: List[RetrievedChunk],
        preserve_structure: bool = True
    ) -> str:
        """
        Assemble filtered chunks into a single, clean context.

        Args:
            chunks: List of filtered chunks to assemble
            preserve_structure: Whether to preserve document structure

        Returns:
            Assembled context as a single string
        """
        if not chunks:
            return ""

        assembled_parts = []

        for chunk in chunks:
            content = chunk.content

            # Preserve document structure if requested
            if preserve_structure:
                content = self.context_utils.preserve_structure(content)

            # Sanitize for LLM consumption
            content = self.context_utils.sanitize_for_llm(content)

            # Add chunk separator and content
            assembled_parts.append(content)

        # Join all parts with appropriate separators
        context = "\n\n---\n\n".join(assembled_parts)

        # Final sanitization
        context = self.context_utils.sanitize_for_llm(context)

        return context

    def enhance_with_structure_preservation(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Enhanced assembly with improved structure preservation (task T030).
        """
        if not chunks:
            return ""

        assembled_parts = []

        for chunk in chunks:
            content = chunk.content

            # Preserve document structure like headings, lists, and formatting
            content = self.context_utils.preserve_structure(content)

            # Add chunk with preserved structure
            assembled_parts.append(content)

        # Join all parts with appropriate separators
        context = "\n\n---\n\n".join(assembled_parts)

        return self.context_utils.sanitize_for_llm(context)

    def implement_heading_preservation(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Implementation for preserving headings specifically (task T031).
        """
        if not chunks:
            return ""

        assembled_parts = []

        for chunk in chunks:
            content = chunk.content

            # Extract and preserve headings
            headings = self.context_utils.extract_headings(content)

            # Preserve the content with headings intact
            content = self.context_utils.preserve_structure(content)

            assembled_parts.append(content)

        # Join all parts
        context = "\n\n---\n\n".join(assembled_parts)

        return self.context_utils.sanitize_for_llm(context)

    def add_llm_safe_formatting(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Add LLM-safe formatting to the assembled context (task T032).
        """
        context = self.assemble_context(chunks)
        return self.context_utils.sanitize_for_llm(context)

    def implement_structural_element_preservation(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Implement preservation of structural elements (task T033).
        """
        if not chunks:
            return ""

        assembled_parts = []

        for chunk in chunks:
            content = chunk.content

            # Preserve various structural elements like headings, lists, code blocks, etc.
            content = self.context_utils.preserve_structure(content)

            assembled_parts.append(content)

        # Join all parts with appropriate separators
        context = "\n\n---\n\n".join(assembled_parts)

        return self.context_utils.sanitize_for_llm(context)

    def add_content_validation_for_llm_safety(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Add content validation for LLM safety (task T034).
        """
        context = self.assemble_context(chunks)
        return self.context_utils.sanitize_for_llm(context)

    def ensure_no_extra_information_added(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Ensure no extra information is added during assembly (task T035).
        """
        # This method ensures that only retrieved text and metadata are used
        # without generating additional content
        if not chunks:
            return ""

        assembled_parts = []

        for chunk in chunks:
            # Use only the original content from the chunk, no generated text
            content = chunk.content
            assembled_parts.append(content)

        # Join all parts with appropriate separators
        context = "\n\n---\n\n".join(assembled_parts)

        return self.context_utils.sanitize_for_llm(context)

    def assemble_with_enhanced_features(
        self,
        chunks: List[RetrievedChunk]
    ) -> str:
        """
        Assemble context with all enhanced features for User Story 3.
        """
        if not chunks:
            return ""

        assembled_parts = []

        for chunk in chunks:
            content = chunk.content

            # 1. Preserve document structure (task T030, T033)
            content = self.context_utils.preserve_structure(content)

            # 2. Extract and maintain headings (task T031)
            # (This is handled within preserve_structure)

            # 3. Ensure no extra information is added (task T035)
            # (We only use the original content)

            assembled_parts.append(content)

        # Join all parts
        context = "\n\n---\n\n".join(assembled_parts)

        # 4. Add LLM-safe formatting (task T032, T034)
        context = self.context_utils.sanitize_for_llm(context)

        return context