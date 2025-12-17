import pytest
from unittest.mock import patch, MagicMock
from src.models.payload import MetadataPayload
from src.models.chunk import BookContentChunk


class TestTraceability:
    def test_payload_allows_traceability_to_source_documents(self, test_settings):
        """Test that metadata payload allows tracing back to source documents."""
        indexing_version = "2025-12-16T23:45:00.123456"

        # Create a payload with comprehensive metadata
        payload = MetadataPayload(
            document_path="docs/modules/module-1/introduction.md",
            section_type="module",
            chapter_name="introduction",
            section_heading="Overview",
            module_name="module-1",
            indexing_version=indexing_version
        )

        payload_dict = payload.to_dict()

        # Verify all traceability fields are present
        assert "document_path" in payload_dict
        assert "section_type" in payload_dict
        assert "chapter_name" in payload_dict
        assert "section_heading" in payload_dict
        assert "module_name" in payload_dict
        assert "indexing_version" in payload_dict

        # Verify specific traceability values
        assert payload_dict["document_path"] == "docs/modules/module-1/introduction.md"
        assert payload_dict["section_type"] == "module"
        assert payload_dict["chapter_name"] == "introduction"
        assert payload_dict["section_heading"] == "Overview"
        assert payload_dict["module_name"] == "module-1"
        assert payload_dict["indexing_version"] == indexing_version

        # Verify we can reconstruct the source location from the metadata
        source_file = payload_dict["document_path"]
        source_module = payload_dict["module_name"]
        source_chapter = payload_dict["chapter_name"]
        source_section = payload_dict["section_heading"]

        # These values should allow us to locate the exact source
        assert source_file.endswith(".md")  # Should be a markdown file
        assert source_module in source_file  # Module name should be in the path
        assert source_chapter in source_file  # Chapter name should be in the path

    def test_chunk_to_payload_traceability(self, test_settings):
        """Test traceability from chunk to payload to source document."""
        # Create a chunk with source information
        chunk = BookContentChunk(
            id="docs/modules/module-2/concepts.md_chunk_0",
            content="This section explains core concepts of the system.",
            document_path="docs/modules/module-2/concepts.md",
            section_type="module",
            module_name="module-2",
            chapter_name="concepts",
            section_heading="Core Concepts"
        )

        # Create payload from chunk metadata
        indexing_version = "2025-12-16T23:45:00.123456"
        payload = MetadataPayload(
            document_path=chunk.document_path,
            section_type=chunk.section_type,
            module_name=chunk.module_name,
            chapter_name=chunk.chapter_name,
            section_heading=chunk.section_heading,
            indexing_version=indexing_version
        )

        # Verify the traceability chain
        payload_dict = payload.to_dict()

        # From payload, we can trace back to:
        # 1. File: document_path
        assert payload_dict["document_path"] == "docs/modules/module-2/concepts.md"

        # 2. Module: module_name
        assert payload_dict["module_name"] == "module-2"

        # 3. Chapter: chapter_name
        assert payload_dict["chapter_name"] == "concepts"

        # 4. Section: section_heading
        assert payload_dict["section_heading"] == "Core Concepts"

        # Verify the original chunk content can be associated with this metadata
        assert "core concepts" in chunk.content.lower()
        assert "system" in chunk.content.lower()

    def test_multiple_chunks_same_document_traceability(self, test_settings):
        """Test traceability for multiple chunks from the same document."""
        document_path = "docs/modules/module-3/advanced-topics.md"
        module_name = "module-3"
        chapter_name = "advanced-topics"

        # Create multiple chunks from the same document but different sections
        chunks_metadata = [
            {
                "id": f"{document_path}_chunk_0",
                "content": "This section covers advanced configuration options.",
                "section_heading": "Configuration"
            },
            {
                "id": f"{document_path}_chunk_1",
                "content": "Advanced deployment strategies are discussed here.",
                "section_heading": "Deployment"
            },
            {
                "id": f"{document_path}_chunk_2",
                "content": "Troubleshooting techniques for common issues.",
                "section_heading": "Troubleshooting"
            }
        ]

        payloads = []
        for chunk_meta in chunks_metadata:
            payload = MetadataPayload(
                document_path=document_path,
                section_type="module",
                module_name=module_name,
                chapter_name=chapter_name,
                section_heading=chunk_meta["section_heading"],
                indexing_version="2025-12-16T23:45:00.123456"
            )
            payloads.append(payload)

        # Verify all payloads trace back to the same document
        for i, payload in enumerate(payloads):
            payload_dict = payload.to_dict()

            # Same document
            assert payload_dict["document_path"] == document_path

            # Same module and chapter
            assert payload_dict["module_name"] == module_name
            assert payload_dict["chapter_name"] == chapter_name

            # Different sections
            expected_section = chunks_metadata[i]["section_heading"]
            assert payload_dict["section_heading"] == expected_section

        # Verify we can distinguish between different sections of the same document
        section_headings = [p.to_dict()["section_heading"] for p in payloads]
        assert len(set(section_headings)) == len(section_headings)  # All unique
        assert "Configuration" in section_headings
        assert "Deployment" in section_headings
        assert "Troubleshooting" in section_headings