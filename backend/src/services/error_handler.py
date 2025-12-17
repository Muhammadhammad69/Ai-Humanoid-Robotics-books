import asyncio
import logging
from typing import Callable, Any, Optional, Dict, TypeVar
from functools import wraps
import time

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

T = TypeVar('T')


def retry_with_exponential_backoff(
    max_retries: int = 3,
    base_delay: float = 1.0,
    max_delay: float = 60.0,
    exceptions: tuple = (Exception,)
):
    """
    Decorator to retry a function with exponential backoff
    """
    def decorator(func: Callable[..., T]) -> Callable[..., T]:
        @wraps(func)
        async def wrapper(*args, **kwargs) -> T:
            last_exception = None

            for attempt in range(max_retries + 1):
                try:
                    return await func(*args, **kwargs)
                except exceptions as e:
                    last_exception = e

                    if attempt == max_retries:
                        # Last attempt, raise the exception
                        logger.error(f"Function {func.__name__} failed after {max_retries} retries: {str(e)}")
                        raise e

                    # Calculate delay with exponential backoff
                    delay = min(base_delay * (2 ** attempt), max_delay)
                    jitter = delay * 0.1  # Add some randomness
                    actual_delay = delay + (jitter * (attempt % 2))  # Alternate small jitter

                    logger.warning(
                        f"Attempt {attempt + 1} failed for {func.__name__}: {str(e)}. "
                        f"Retrying in {actual_delay:.2f} seconds..."
                    )

                    await asyncio.sleep(actual_delay)

            # This line should never be reached, but included for type safety
            raise last_exception  # type: ignore

        return wrapper

    return decorator


class LLMErrorHandler:
    """
    Error handling utilities for LLM API calls
    """

    def __init__(self):
        self.retry_attempts = 3

    async def handle_llm_api_call(
        self,
        api_call_func: Callable,
        *args,
        **kwargs
    ) -> Dict[str, Any]:
        """
        Handle LLM API calls with comprehensive error handling and retry logic
        """
        try:
            # Execute the API call with retry logic
            result = await self._execute_with_retry(api_call_func, *args, **kwargs)
            return {
                "success": True,
                "data": result,
                "error": None
            }
        except Exception as e:
            logger.error(f"LLM API call failed: {str(e)}")
            return {
                "success": False,
                "data": None,
                "error": str(e),
                "error_type": type(e).__name__
            }

    @retry_with_exponential_backoff(
        max_retries=3,
        base_delay=1.0,
        max_delay=30.0,
        exceptions=(Exception,)
    )
    async def _execute_with_retry(self, api_call_func: Callable, *args, **kwargs):
        """
        Execute an API call with retry logic
        """
        start_time = time.time()
        result = await api_call_func(*args, **kwargs)
        execution_time = time.time() - start_time

        logger.info(f"LLM API call succeeded in {execution_time:.2f} seconds")
        return result

    def classify_error(self, error: Exception) -> str:
        """
        Classify the type of error for appropriate handling
        """
        error_str = str(error).lower()

        if "rate limit" in error_str or "quota" in error_str or "exceeded" in error_str:
            return "RATE_LIMIT"
        elif "api key" in error_str or "authentication" in error_str or "unauthorized" in error_str:
            return "AUTHENTICATION"
        elif "not found" in error_str or "404" in error_str:
            return "NOT_FOUND"
        elif "timeout" in error_str or "connection" in error_str:
            return "CONNECTION"
        elif "invalid" in error_str or "malformed" in error_str:
            return "VALIDATION"
        else:
            return "UNKNOWN"

    def generate_fallback_response(self, error_type: str) -> str:
        """
        Generate appropriate fallback response based on error type
        """
        fallback_responses = {
            "RATE_LIMIT": "The system is currently experiencing high demand. Please try again in a moment.",
            "AUTHENTICATION": "There was an issue with the AI service configuration. Our team has been notified.",
            "NOT_FOUND": "The requested AI model is not available. Please try again later.",
            "CONNECTION": "Could not connect to the AI service. Please try again.",
            "VALIDATION": "The request could not be processed due to invalid parameters.",
            "UNKNOWN": "An unexpected error occurred. Please try again or contact support."
        }

        return fallback_responses.get(error_type, "An error occurred while processing your request.")


# Global instance for use in other modules
llm_error_handler = LLMErrorHandler()