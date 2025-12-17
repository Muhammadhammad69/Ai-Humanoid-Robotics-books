import pytest
from src.services.chunker import BasicChunker
from src.models.chunk import BookContentChunk


class TestBasicChunker:
    def test_chunk_content_splits_long_content(self, test_settings):
        """Test that chunk_content splits long content into smaller chunks."""
        chunker = BasicChunker(max_chunk_size=50)  # Small size for testing

        long_content = "This is a long piece of content that should be split into multiple chunks. " * 5
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Section"
        }

        chunks = chunker.chunk_content(long_content, document_info)

        # Should have more than one chunk
        assert len(chunks) > 1

        # Each chunk should have content
        for chunk in chunks:
            assert len(chunk.content) > 0
            assert len(chunk.content) <= 50  # max_chunk_size

        # All chunks should have the same document info
        for chunk in chunks:
            assert chunk.document_path == "docs/test.md"
            assert chunk.section_type == "intro"
            assert chunk.chapter_name == "test"

    def test_chunk_content_handles_short_content(self, test_settings):
        """Test that chunk_content handles content shorter than max size."""
        chunker = BasicChunker(max_chunk_size=1000)  # Large size

        short_content = "This is a short piece of content."
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Section"
        }

        chunks = chunker.chunk_content(short_content, document_info)

        # Should have exactly one chunk
        assert len(chunks) == 1

        # The chunk should contain the full content
        assert chunks[0].content == short_content
        assert chunks[0].document_path == "docs/test.md"

    def test_chunk_content_preserves_document_info(self, test_settings):
        """Test that chunk_content preserves document information."""
        chunker = BasicChunker(max_chunk_size=100)

        content = "Content 1. Content 2. Content 3."
        document_info = {
            "document_path": "docs/module-1/lesson.md",
            "section_type": "module",
            "module_name": "module-1",
            "chapter_name": "lesson",
            "section_heading": "Lesson Title"
        }

        chunks = chunker.chunk_content(content, document_info)

        # All chunks should preserve the document info
        for chunk in chunks:
            assert chunk.document_path == "docs/module-1/lesson.md"
            assert chunk.section_type == "module"
            assert chunk.module_name == "module-1"
            assert chunk.chapter_name == "lesson"
            assert chunk.section_heading == "Lesson Title"
            assert chunk.language == "en"
            assert chunk.source_type == "book"

    def test_chunk_content_creates_unique_ids(self, test_settings):
        """Test that chunk_content creates unique IDs for each chunk."""
        chunker = BasicChunker(max_chunk_size=10)

        long_content = "This is content that will be split into multiple chunks."
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Section"
        }

        chunks = chunker.chunk_content(long_content, document_info)

        # Each chunk should have a unique ID
        ids = [chunk.id for chunk in chunks]
        assert len(ids) == len(set(ids))  # All IDs should be unique

    def test_chunk_content_preserving_semantics(self, test_settings):
        """Test the enhanced semantic chunking functionality."""
        chunker = BasicChunker(max_chunk_size=100)

        # Content with markdown headings
        content_with_headings = """# Introduction

This is the introduction section with some content.

## Getting Started

Here's how to get started with the system.

### Prerequisites

- Python 3.11+
- uv package manager

## Main Concepts

These are the main concepts you need to understand.
"""
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Document"
        }

        chunks = chunker.chunk_content_preserving_semantics(content_with_headings, document_info)

        # Should have multiple chunks based on headings
        assert len(chunks) >= 3  # At least one for each heading section

        # Check that chunks have proper heading levels
        has_h1_chunk = any(chunk.heading_level == 1 for chunk in chunks)
        has_h2_chunk = any(chunk.heading_level == 2 for chunk in chunks)
        has_h3_chunk = any(chunk.heading_level == 3 for chunk in chunks)

        assert has_h1_chunk or has_h2_chunk or has_h3_chunk  # At least some chunks should have heading levels

        # All chunks should have proper document info
        for chunk in chunks:
            assert chunk.document_path == "docs/test.md"
            assert chunk.section_type == "intro"
            assert chunk.chapter_name == "test"
            assert len(chunk.content) > 0

    def test_chunk_content_preserving_semantics_no_headings(self, test_settings):
        """Test enhanced chunking with content that has no headings."""
        chunker = BasicChunker(max_chunk_size=50)

        content_without_headings = "This is content without any markdown headings. " * 3
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Section"
        }

        chunks = chunker.chunk_content_preserving_semantics(content_without_headings, document_info)

        # Should still chunk the content even without headings
        assert len(chunks) >= 1

        for chunk in chunks:
            assert len(chunk.content) <= 50  # max_chunk_size
            assert chunk.document_path == "docs/test.md"
            assert chunk.section_type == "intro"
            assert chunk.chapter_name == "test"
            assert chunk.heading_level is None  # No headings means no heading level