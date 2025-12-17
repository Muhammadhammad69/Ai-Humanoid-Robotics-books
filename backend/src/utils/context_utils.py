import re
from typing import List, Dict, Any
from ..models.query import RetrievedChunk


class ContextUtils:
    """
    Utility functions for context processing and assembly.
    """

    @staticmethod
    def preserve_structure(content: str) -> str:
        """
        Preserve document structure like headings, lists, and formatting.

        Args:
            content: Raw content to process

        Returns:
            Content with structure preserved
        """
        # This is an enhanced implementation for tasks T030 and T033
        if not content:
            return content

        # Ensure proper spacing around markdown headings (#, ##, ###, etc.)
        content = re.sub(r'(\n)(#{1,6}\s+)', r'\1\2', content)

        # Ensure proper spacing around ATX-style headings
        content = re.sub(r'(\n)(\s*#{1,6}\s+)', r'\1\2', content)

        # Ensure proper spacing around setext-style headings (underlined)
        lines = content.split('\n')
        processed_lines = []
        i = 0
        while i < len(lines):
            line = lines[i]
            # Check if this line and the next form a setext heading
            if (i + 1 < len(lines) and
                len(line.strip()) > 0 and
                len(lines[i + 1].strip()) > 0 and
                all(c in ['=', '-'] for c in lines[i + 1].strip())):
                # This is a setext heading, preserve it
                processed_lines.append(line)
                processed_lines.append(lines[i + 1])
                i += 2
            else:
                processed_lines.append(line)
                i += 1

        content = '\n'.join(processed_lines)

        # Ensure proper spacing around list items (both unordered and ordered)
        content = re.sub(r'(\n)([*+\-]\s+)', r'\1\2', content)  # Unordered lists
        content = re.sub(r'(\n)(\d+\.\s+)', r'\1\2', content)  # Ordered lists

        # Ensure proper spacing around blockquotes
        content = re.sub(r'(\n)(>\s+)', r'\1\2', content)

        # Ensure proper spacing around code blocks
        content = re.sub(r'(\n)(```)', r'\1\2', content)

        # Normalize whitespace while preserving structure
        content = re.sub(r'\n\s*\n', '\n\n', content)  # Multiple blank lines to single

        return content.strip()

    @staticmethod
    def sanitize_for_llm(content: str) -> str:
        """
        Sanitize content to make it safe for LLM consumption.

        Args:
            content: Content to sanitize

        Returns:
            Sanitized content safe for LLM
        """
        if not content:
            return content

        # Remove potentially problematic sequences
        sanitized = content.replace('\x00', '')  # Remove null bytes
        sanitized = sanitized.replace('\ufffd', '')  # Remove replacement characters

        # Ensure proper line endings
        sanitized = sanitized.replace('\r\n', '\n').replace('\r', '\n')

        # Basic XSS prevention - remove script tags (if any)
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove any control characters except common whitespace
        sanitized = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', ' ', sanitized)

        return sanitized.strip()

    @staticmethod
    def calculate_similarity(text1: str, text2: str) -> float:
        """
        Calculate similarity between two text chunks to detect duplicates.

        Args:
            text1: First text chunk
            text2: Second text chunk

        Returns:
            Similarity score between 0.0 and 1.0
        """
        if not text1 or not text2:
            return 0.0 if text1 != text2 else 1.0

        # Simple Jaccard similarity based on words
        words1 = set(text1.lower().split())
        words2 = set(text2.lower().split())

        intersection = words1.intersection(words2)
        union = words1.union(words2)

        if not union:
            return 1.0 if not intersection else 0.0

        return len(intersection) / len(union)

    @staticmethod
    def remove_duplicates(chunks: List[RetrievedChunk], threshold: float = 0.9) -> List[RetrievedChunk]:
        """
        Remove duplicate or near-duplicate chunks based on content similarity.

        Args:
            chunks: List of retrieved chunks
            threshold: Similarity threshold above which chunks are considered duplicates

        Returns:
            List of chunks with duplicates removed
        """
        if not chunks:
            return []

        unique_chunks = [chunks[0]]  # Always keep the first chunk

        for chunk in chunks[1:]:
            is_duplicate = False
            for existing_chunk in unique_chunks:
                similarity = ContextUtils.calculate_similarity(chunk.content, existing_chunk.content)
                if similarity >= threshold:
                    is_duplicate = True
                    break

            if not is_duplicate:
                unique_chunks.append(chunk)

        return unique_chunks

    @staticmethod
    def extract_headings(content: str) -> List[str]:
        """
        Extract headings from content to preserve document structure.

        Args:
            content: Content to extract headings from

        Returns:
            List of headings found in the content
        """
        if not content:
            return []

        # Match markdown-style headings (#, ##, ###, etc.)
        heading_pattern = r'^(#{1,6})\s+(.+)$'
        lines = content.split('\n')
        headings = []

        for line in lines:
            match = re.match(heading_pattern, line.strip())
            if match:
                headings.append(match.group(2).strip())

        return headings