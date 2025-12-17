import re
from typing import List, Dict, Any, Optional
from ..models.chunk import BookContentChunk


class ContextFormatterService:
    """
    Service for preparing context data in LLM-safe format
    """

    def __init__(self):
        pass

    def format_context_for_llm(
        self,
        chunks: List[Dict[str, Any]],
        max_token_limit: Optional[int] = 25000  # Approximate token limit for Gemini
    ) -> str:
        """
        Format retrieved chunks into a context string suitable for LLM consumption
        """
        formatted_chunks = []

        for chunk in chunks:
            # Handle different possible formats of chunks
            if isinstance(chunk, dict):
                content = chunk.get('content', '')
                metadata = chunk.get('metadata', {})
            else:
                # Assume it's a Chunk object or similar
                content = getattr(chunk, 'content', '')
                metadata = getattr(chunk, 'metadata', {})

            if content.strip():
                # Add metadata information if available
                chunk_text = content
                if metadata:
                    meta_info = []
                    for key, value in metadata.items():
                        if key not in ['content', 'score']:  # Skip content and score as they're handled separately
                            meta_info.append(f"{key}: {value}")
                    if meta_info:
                        chunk_text = f"[{', '.join(meta_info)}]\n{content}"

                formatted_chunks.append(chunk_text)

        # Combine all chunks with separators
        context = "\n\n".join(formatted_chunks)

        # Truncate if necessary to stay within token limits
        if max_token_limit:
            context = self._truncate_to_token_limit(context, max_token_limit)

        return context

    def _truncate_to_token_limit(self, text: str, max_tokens: int) -> str:
        """
        Truncate text to stay within token limit (approximate - using word count as proxy)
        """
        # This is a rough approximation - 1 token is roughly 4 characters or 0.75 words
        # For more accuracy, we could use a proper tokenizer
        words = text.split()

        if len(words) > max_tokens:
            # Keep the first max_tokens words and add an indicator that it was truncated
            truncated_words = words[:max_tokens]
            return " ".join(truncated_words) + " [Context truncated due to length limitations]"

        return text

    def validate_context_safety(self, context: str) -> Dict[str, Any]:
        """
        Validate context for safety issues and return validation results
        """
        issues = []

        # Check for potentially sensitive information
        sensitive_patterns = [
            r'\b(?:password|pwd|pass)\s*[:=]\s*\S+',
            r'\b(?:api[_-]?key|secret|token)\s*[:=]\s*\S+',
            r'\b\d{3}-\d{2}-\d{4}\b',  # SSN pattern
            r'\b\d{16}\b',  # Basic credit card pattern
            r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b'  # Email (might be ok in some contexts)
        ]

        for pattern in sensitive_patterns:
            if re.search(pattern, context, re.IGNORECASE):
                issues.append(f"Potential sensitive information detected: {pattern}")

        # Check for code injection patterns
        injection_patterns = [
            r'<script',
            r'javascript:',
            r'vbscript:',
            r'on\w+\s*=',
            r'eval\(',
            r'exec\('
        ]

        for pattern in injection_patterns:
            if re.search(pattern, context, re.IGNORECASE):
                issues.append(f"Potential code injection pattern detected: {pattern}")

        return {
            "is_safe": len(issues) == 0,
            "issues": issues,
            "context_length": len(context)
        }

    def filter_sensitive_content(self, context: str) -> str:
        """
        Filter out sensitive content from context before LLM processing
        """
        # Remove common sensitive patterns (replace with placeholders)
        filtered_context = re.sub(
            r'\b(?:password|pwd|pass)\s*[:=]\s*\S+',
            '[REDACTED_PASSWORD]',
            context,
            flags=re.IGNORECASE
        )

        filtered_context = re.sub(
            r'\b(?:api[_-]?key|secret|token)\s*[:=]\s*\S+',
            '[REDACTED_API_KEY]',
            filtered_context,
            flags=re.IGNORECASE
        )

        # Remove potential SSNs
        filtered_context = re.sub(
            r'\b\d{3}-\d{2}-\d{4}\b',
            '[REDACTED_SSN]',
            filtered_context
        )

        # Remove potential credit card numbers
        filtered_context = re.sub(
            r'\b\d{4}[\s-]?\d{4}[\s-]?\d{4}[\s-]?\d{4}\b',
            '[REDACTED_CC]',
            filtered_context
        )

        return filtered_context


# Global instance for use in other modules
context_formatter_service = ContextFormatterService()