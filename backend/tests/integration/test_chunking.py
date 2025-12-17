import pytest
from src.services.chunker import BasicChunker
from src.services.content_ingestor import ContentIngestor
from src.utils.markdown_parser import extract_headings


class TestSemanticChunking:
    def test_content_ingestor_uses_semantic_chunking(self, test_settings, sample_markdown_content):
        """Test that content ingestor properly uses semantic chunking."""
        ingestor = ContentIngestor(docs_path=test_settings.DOCS_PATH)

        # Count chunks with semantic chunking enabled
        semantic_chunks = list(ingestor.process_documents(use_semantic_chunking=True))

        # Count chunks with semantic chunking disabled (should be fewer)
        simple_chunks = list(ingestor.process_documents(use_semantic_chunking=False))

        # With semantic chunking, we should get more granular chunks
        # (though this depends on the specific sample content)
        assert len(semantic_chunks) >= len(simple_chunks)

        # Verify that chunks have proper heading information when using semantic chunking
        for chunk in semantic_chunks:
            assert hasattr(chunk, 'section_heading')
            assert chunk.section_heading is not None

    def test_chunker_respects_chunk_size_limit(self, test_settings):
        """Test that the chunker respects the maximum chunk size."""
        chunker = BasicChunker(max_chunk_size=100)  # Small size for testing

        # Create content that will definitely exceed the chunk size
        long_content = "This is a test sentence. " * 50  # 50 sentences
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Section"
        }

        chunks = chunker.chunk_content_preserving_semantics(long_content, document_info)

        # All chunks should respect the size limit
        for chunk in chunks:
            assert len(chunk.content) <= 100  # max_chunk_size

        # Should have multiple chunks due to the size limit
        assert len(chunks) > 1

    def test_chunker_preserves_semantic_boundaries(self, test_settings):
        """Test that the chunker preserves semantic boundaries like headings."""
        chunker = BasicChunker(max_chunk_size=200)  # Size that allows some flexibility

        # Content with clear semantic boundaries
        content_with_boundaries = """# Introduction

This is the introduction section with a few sentences to provide context.

## Getting Started

Here's how to get started with the system. This section has more detailed information that should stay together in a chunk.

### Prerequisites

Before you begin, make sure you have the following:

- Python 3.11+
- uv package manager

## Main Concepts

These are the main concepts you need to understand. Each concept builds on the previous one.
"""
        document_info = {
            "document_path": "docs/test.md",
            "section_type": "intro",
            "chapter_name": "test",
            "section_heading": "Test Document"
        }

        chunks = chunker.chunk_content_preserving_semantics(content_with_boundaries, document_info)

        # Should have chunks that respect heading boundaries
        assert len(chunks) >= 3  # At least one chunk per major heading

        # Verify that chunks contain meaningful content
        for chunk in chunks:
            assert len(chunk.content.strip()) > 0
            assert chunk.section_heading is not None

        # Check that headings are preserved in chunk metadata
        has_intro_chunk = any("Introduction" in chunk.section_heading for chunk in chunks)
        has_getting_started_chunk = any("Getting Started" in chunk.section_heading for chunk in chunks)
        has_main_concepts_chunk = any("Main Concepts" in chunk.section_heading for chunk in chunks)

        assert has_intro_chunk or has_getting_started_chunk or has_main_concepts_chunk

    def test_heading_extraction_and_chunking_integration(self, test_settings):
        """Test the integration between heading extraction and chunking."""
        from src.utils.markdown_parser import extract_headings, split_by_headings

        content = """# Main Topic

Some introductory content.

## Subtopic 1

Content related to subtopic 1.

## Subtopic 2

Different content for subtopic 2.

### Detailed Section

Very detailed information here.
"""

        # Extract headings
        headings = extract_headings(content)
        assert len(headings) == 4  # Main Topic (H1), Subtopic 1 (H2), Subtopic 2 (H2), Detailed Section (H3)

        # Verify heading levels
        levels = [h.level for h in headings]
        assert 1 in levels  # Should have H1
        assert 2 in levels  # Should have H2
        assert 3 in levels  # Should have H3

        # Split by headings
        heading_content_pairs = split_by_headings(content, headings)

        # Should have pairs for each heading and the content after it
        assert len(heading_content_pairs) >= 3  # At least one pair per heading