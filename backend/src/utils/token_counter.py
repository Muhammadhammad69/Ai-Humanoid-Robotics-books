import re
from typing import Union, List


class TokenCounter:
    """
    Utility for counting tokens in text content.
    Uses a simple approach that approximates token count based on word segmentation.
    For more accurate token counting, integrate with specific tokenizer for your LLM.
    """

    @staticmethod
    def count_tokens(text: str) -> int:
        """
        Count approximate number of tokens in the given text.

        Args:
            text: Input text to count tokens for

        Returns:
            Approximate number of tokens
        """
        if not text:
            return 0

        # Simple tokenization: split on whitespace and punctuation
        # This is a rough approximation - for more accuracy, use a specific tokenizer
        # like tiktoken for OpenAI models or transformers tokenizer
        tokens = re.findall(r'\b\w+\b|[^\w\s]', text)
        return len(tokens)

    @staticmethod
    def count_tokens_batch(texts: List[str]) -> List[int]:
        """
        Count tokens for a batch of texts.

        Args:
            texts: List of input texts to count tokens for

        Returns:
            List of token counts corresponding to each input text
        """
        return [TokenCounter.count_tokens(text) for text in texts]

    @staticmethod
    def truncate_to_token_budget(text: str, token_budget: int) -> str:
        """
        Truncate text to fit within the specified token budget.

        Args:
            text: Input text to truncate
            token_budget: Maximum number of tokens allowed

        Returns:
            Truncated text that fits within the token budget
        """
        if not text or token_budget <= 0:
            return ""

        # Simple truncation by words
        words = text.split()
        current_tokens = 0
        result_words = []

        for word in words:
            # Estimate tokens for this word
            word_tokens = TokenCounter.count_tokens(word)

            if current_tokens + word_tokens <= token_budget:
                result_words.append(word)
                current_tokens += word_tokens
            else:
                break

        return " ".join(result_words)

    @staticmethod
    def fits_in_token_budget(text: str, token_budget: int) -> bool:
        """
        Check if text fits within the specified token budget.

        Args:
            text: Input text to check
            token_budget: Maximum number of tokens allowed

        Returns:
            True if text fits within budget, False otherwise
        """
        return TokenCounter.count_tokens(text) <= token_budget